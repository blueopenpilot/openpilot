#!/usr/bin/env python3

# Safe Update: A simple service that waits for network access and tries to
# update every 10 minutes. It's intended to make the OP update process more
# robust against Git repository corruption. This service DOES NOT try to fix
# an already-corrupt BASEDIR Git repo, only prevent it from happening.
#
# During normal operation, both onroad and offroad, the update process makes
# no changes to the BASEDIR install of OP. All update attempts are performed
# in a disposable staging area provided by OverlayFS. It assumes the deleter
# process provides enough disk space to carry out the process.
#
# If an update succeeds, a flag is set, and the update is swapped in at the
# next reboot. If an update is interrupted or otherwise fails, the OverlayFS
# upper layer and metadata can be discarded before trying again.
#
# The swap on boot is triggered by launch_chffrplus.sh
# gated on the existence of $FINALIZED/.overlay_consistent and also the
# existence and mtime of $BASEDIR/.overlay_init.
#
# Other than build byproducts, BASEDIR should not be modified while this
# service is running. Developers modifying code directly in BASEDIR should
# disable this service.

import os
import datetime
import subprocess
import psutil
import shutil
import signal
import fcntl
import time
import threading
from pathlib import Path
from typing import List, Tuple, Optional
from markdown_it import MarkdownIt

from common.basedir import BASEDIR
from common.params import Params
from system.hardware import AGNOS, HARDWARE
from system.swaglog import cloudlog
from selfdrive.controls.lib.alertmanager import set_offroad_alert
from system.version import is_tested_branch

LOCK_FILE = os.getenv("UPDATER_LOCK_FILE", "/tmp/safe_staging_overlay.lock")
STAGING_ROOT = os.getenv("UPDATER_STAGING_ROOT", "/data/safe_staging")

OVERLAY_UPPER = os.path.join(STAGING_ROOT, "upper")
OVERLAY_METADATA = os.path.join(STAGING_ROOT, "metadata")
OVERLAY_MERGED = os.path.join(STAGING_ROOT, "merged")
FINALIZED = os.path.join(STAGING_ROOT, "finalized")

DAYS_NO_CONNECTIVITY_MAX = 14     # do not allow to engage after this many days
DAYS_NO_CONNECTIVITY_PROMPT = 10  # send an offroad prompt after this many days

class WaitTimeHelper:
  def __init__(self, proc):
    self.proc = proc
    self.ready_event = threading.Event()
    self.shutdown = False
    signal.signal(signal.SIGTERM, self.graceful_shutdown)
    signal.signal(signal.SIGINT, self.graceful_shutdown)
    signal.signal(signal.SIGHUP, self.update_now)

  def graceful_shutdown(self, signum: int, frame) -> None:
    # umount -f doesn't appear effective in avoiding "device busy" on NEOS,
    # so don't actually die until the next convenient opportunity in main().
    cloudlog.info("caught SIGINT/SIGTERM, dismounting overlay at next opportunity")

    # forward the signal to all our child processes
    child_procs = self.proc.children(recursive=True)
    for p in child_procs:
      p.send_signal(signum)

    self.shutdown = True
    self.ready_event.set()

  def update_now(self, signum: int, frame) -> None:
    cloudlog.info("caught SIGHUP, running update check immediately")
    self.ready_event.set()

  def sleep(self, t: float) -> None:
    self.ready_event.wait(timeout=t)


def run(cmd: List[str], cwd: Optional[str] = None, low_priority: bool = False):
  if low_priority:
    cmd = ["nice", "-n", "19"] + cmd
  return subprocess.check_output(cmd, cwd=cwd, stderr=subprocess.STDOUT, encoding='utf8')


def set_consistent_flag(consistent: bool) -> None:
  os.sync()
  consistent_file = Path(os.path.join(FINALIZED, ".overlay_consistent"))
  if consistent:
    consistent_file.touch()
  elif not consistent:
    consistent_file.unlink(missing_ok=True)
  os.sync()


def set_params(new_version: bool, failed_count: int, exception: Optional[str]) -> None:
  params = Params()

  params.put("UpdateFailedCount", str(failed_count))

  last_update = datetime.datetime.utcnow()
  if failed_count == 0:
    t = last_update.isoformat()
    params.put("LastUpdateTime", t.encode('utf8'))
  else:
    try:
      t = params.get("LastUpdateTime", encoding='utf8')
      last_update = datetime.datetime.fromisoformat(t)
    except (TypeError, ValueError):
      pass

  if exception is None:
    params.delete("LastUpdateException")
  else:
    params.put("LastUpdateException", exception)

  # Write out release notes for new versions
  if new_version:
    try:
      with open(os.path.join(FINALIZED, "RELEASES.md"), "rb") as f:
        r = f.read().split(b'\n\n', 1)[0]  # Slice latest release notes
      try:
        params.put("ReleaseNotes", MarkdownIt().render(r.decode("utf-8")))
      except Exception:
        params.put("ReleaseNotes", r + b"\n")
    except Exception:
      params.put("ReleaseNotes", "")
    params.put_bool("UpdateAvailable", True)

  # Handle user prompt
  for alert in ("Offroad_UpdateFailed", "Offroad_ConnectivityNeeded", "Offroad_ConnectivityNeededPrompt"):
    set_offroad_alert(alert, False)

  now = datetime.datetime.utcnow()
  dt = now - last_update
  if failed_count > 15 and exception is not None:
    if is_tested_branch():
      extra_text = "Ensure the software is correctly installed"
    else:
      extra_text = exception
    set_offroad_alert("Offroad_UpdateFailed", True, extra_text=extra_text)
  elif dt.days > DAYS_NO_CONNECTIVITY_MAX and failed_count > 1:
    set_offroad_alert("Offroad_ConnectivityNeeded", True)
  elif dt.days > DAYS_NO_CONNECTIVITY_PROMPT:
    remaining = max(DAYS_NO_CONNECTIVITY_MAX - dt.days, 1)
    set_offroad_alert("Offroad_ConnectivityNeededPrompt", True, extra_text=f"{remaining} day{'' if remaining == 1 else 's'}.")


def setup_git_options(cwd: str) -> None:
  # We sync FS object atimes (which NEOS doesn't use) and mtimes, but ctimes
  # are outside user control. Make sure Git is set up to ignore system ctimes,
  # because they change when we make hard links during finalize. Otherwise,
  # there is a lot of unnecessary churn. This appears to be a common need on
  # OSX as well: https://www.git-tower.com/blog/make-git-rebase-safe-on-osx/

  # We are using copytree to copy the directory, which also changes
  # inode numbers. Ignore those changes too.

  # Set protocol to the new version (default after git 2.26) to reduce data
  # usage on git fetch --dry-run from about 400KB to 18KB.
  git_cfg = [
    ("core.trustctime", "false"),
    ("core.checkStat", "minimal"),
    ("protocol.version", "2"),
    ("gc.auto", "0"),
    ("gc.autoDetach", "false"),
  ]
  for option, value in git_cfg:
    run(["git", "config", option, value], cwd)


def dismount_overlay() -> None:
  if os.path.ismount(OVERLAY_MERGED):
    cloudlog.info("unmounting existing overlay")
    run(["sudo", "umount", "-l", OVERLAY_MERGED])


def init_overlay() -> None:

  overlay_init_file = Path(os.path.join(BASEDIR, ".overlay_init"))

  # Re-create the overlay if BASEDIR/.git has changed since we created the overlay
  if overlay_init_file.is_file():
    git_dir_path = os.path.join(BASEDIR, ".git")
    new_files = run(["find", git_dir_path, "-newer", str(overlay_init_file)])
    if not len(new_files.splitlines()):
      # A valid overlay already exists
      return
    else:
      cloudlog.info(".git directory changed, recreating overlay")

  cloudlog.info("preparing new safe staging area")

  params = Params()
  params.put_bool("UpdateAvailable", False)
  set_consistent_flag(False)
  dismount_overlay()
  run(["sudo", "rm", "-rf", STAGING_ROOT])
  if os.path.isdir(STAGING_ROOT):
    shutil.rmtree(STAGING_ROOT)

  for dirname in [STAGING_ROOT, OVERLAY_UPPER, OVERLAY_METADATA, OVERLAY_MERGED]:
    os.mkdir(dirname, 0o755)

  if os.lstat(BASEDIR).st_dev != os.lstat(OVERLAY_MERGED).st_dev:
    raise RuntimeError("base and overlay merge directories are on different filesystems; not valid for overlay FS!")

  # Leave a timestamped canary in BASEDIR to check at startup. The device clock
  # should be correct by the time we get here. If the init file disappears, or
  # critical mtimes in BASEDIR are newer than .overlay_init, continue.sh can
  # assume that BASEDIR has used for local development or otherwise modified,
  # and skips the update activation attempt.
  consistent_file = Path(os.path.join(BASEDIR, ".overlay_consistent"))
  if consistent_file.is_file():
    consistent_file.unlink()
  overlay_init_file.touch()

  os.sync()
  overlay_opts = f"lowerdir={BASEDIR},upperdir={OVERLAY_UPPER},workdir={OVERLAY_METADATA}"

  mount_cmd = ["mount", "-t", "overlay", "-o", overlay_opts, "none", OVERLAY_MERGED]
  run(["sudo"] + mount_cmd)
  run(["sudo", "chmod", "755", os.path.join(OVERLAY_METADATA, "work")])

  git_diff = run(["git", "diff"], OVERLAY_MERGED, low_priority=True)
  params.put("GitDiff", git_diff)
  cloudlog.info(f"git diff output:\n{git_diff}")


def finalize_update(wait_helper: WaitTimeHelper) -> None:
  """Take the current OverlayFS merged view and finalize a copy outside of
  OverlayFS, ready to be swapped-in at BASEDIR. Copy using shutil.copytree"""

  # Remove the update ready flag and any old updates
  cloudlog.info("creating finalized version of the overlay")
  set_consistent_flag(False)

  # Copy the merged overlay view and set the update ready flag
  if os.path.exists(FINALIZED):
    shutil.rmtree(FINALIZED)
  shutil.copytree(OVERLAY_MERGED, FINALIZED, symlinks=True)

  run(["git", "reset", "--hard"], FINALIZED)
  run(["git", "submodule", "foreach", "--recursive", "git", "reset"], FINALIZED)

  cloudlog.info("Starting git gc")
  t = time.monotonic()
  try:
    run(["git", "gc"], FINALIZED)
    cloudlog.event("Done git gc", duration=time.monotonic() - t)
  except subprocess.CalledProcessError:
    cloudlog.exception(f"Failed git gc, took {time.monotonic() - t:.3f} s")

  if wait_helper.shutdown:
    cloudlog.info("got interrupted finalizing overlay")
  else:
    set_consistent_flag(True)
    cloudlog.info("done finalizing overlay")


def handle_agnos_update(wait_helper: WaitTimeHelper) -> None:
  from system.hardware.tici.agnos import flash_agnos_update, get_target_slot_number

  cur_version = HARDWARE.get_os_version()
  updated_version = run(["bash", "-c", r"unset AGNOS_VERSION && source launch_env.sh && \
                          echo -n $AGNOS_VERSION"], OVERLAY_MERGED).strip()

  cloudlog.info(f"AGNOS version check: {cur_version} vs {updated_version}")
  if cur_version == updated_version:
    return

  # prevent an openpilot getting swapped in with a mismatched or partially downloaded agnos
  set_consistent_flag(False)

  cloudlog.info(f"Beginning background installation for AGNOS {updated_version}")
  set_offroad_alert("Offroad_NeosUpdate", True)

  manifest_path = os.path.join(OVERLAY_MERGED, "system/hardware/tici/agnos.json")
  target_slot_number = get_target_slot_number()
  flash_agnos_update(manifest_path, target_slot_number, cloudlog)
  set_offroad_alert("Offroad_NeosUpdate", False)


def check_git_fetch_result(fetch_txt: str) -> bool:
  err_msg = "Failed to add the host to the list of known hosts (/data/data/com.termux/files/home/.ssh/known_hosts).\n"
  return len(fetch_txt) > 0 and (fetch_txt != err_msg)


def check_for_update() -> Tuple[bool, bool]:
  setup_git_options(OVERLAY_MERGED)
  try:
    git_fetch_output = run(["git", "fetch", "--dry-run"], OVERLAY_MERGED, low_priority=True)
    return True, check_git_fetch_result(git_fetch_output)
  except subprocess.CalledProcessError:
    return False, False


def fetch_update(wait_helper: WaitTimeHelper) -> bool:
  cloudlog.info("attempting git fetch inside staging overlay")

  setup_git_options(OVERLAY_MERGED)

  git_fetch_output = run(["git", "fetch"], OVERLAY_MERGED, low_priority=True)
  cloudlog.info("git fetch success: %s", git_fetch_output)

  cur_hash = run(["git", "rev-parse", "HEAD"], OVERLAY_MERGED).rstrip()
  upstream_hash = run(["git", "rev-parse", "@{u}"], OVERLAY_MERGED).rstrip()
  new_version: bool = cur_hash != upstream_hash
  git_fetch_result = check_git_fetch_result(git_fetch_output)

  cloudlog.info(f"comparing {cur_hash} to {upstream_hash}")
  if new_version or git_fetch_result:
    cloudlog.info("Running update")

    if new_version:
      cloudlog.info("git reset in progress")
      r = [
        run(["git", "reset", "--hard", "@{u}"], OVERLAY_MERGED, low_priority=True),
        run(["git", "clean", "-xdf"], OVERLAY_MERGED, low_priority=True ),
        run(["git", "submodule", "init"], OVERLAY_MERGED, low_priority=True),
        run(["git", "submodule", "update"], OVERLAY_MERGED, low_priority=True),
      ]
      cloudlog.info("git reset success: %s", '\n'.join(r))

      if AGNOS:
        handle_agnos_update(wait_helper)

    # Create the finalized, ready-to-swap update
    finalize_update(wait_helper)
    cloudlog.info("openpilot update successful!")
  else:
    cloudlog.info("nothing new from git at this time")

  return new_version


def main() -> None:
  params = Params()

  if params.get_bool("DisableUpdates"):
    cloudlog.warning("updates are disabled by the DisableUpdates param")
    exit(0)

  ov_lock_fd = open(LOCK_FILE, 'w')
  try:
    fcntl.flock(ov_lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
  except OSError as e:
    raise RuntimeError("couldn't get overlay lock; is another instance running?") from e

  # Set low io priority
  proc = psutil.Process()
  if psutil.LINUX:
    proc.ionice(psutil.IOPRIO_CLASS_BE, value=7)

  # Check if we just performed an update
  if Path(os.path.join(STAGING_ROOT, "old_openpilot")).is_dir():
    cloudlog.event("update installed")

  if not params.get("InstallDate"):
    t = datetime.datetime.utcnow().isoformat()
    params.put("InstallDate", t.encode('utf8'))

  overlay_init = Path(os.path.join(BASEDIR, ".overlay_init"))
  overlay_init.unlink(missing_ok=True)

  update_failed_count = 0  # TODO: Load from param?
  wait_helper = WaitTimeHelper(proc)

  # Run the update loop
  while not wait_helper.shutdown:
    wait_helper.ready_event.clear()

    # Attempt an update
    exception = None
    new_version = False
    update_failed_count += 1
    try:
      init_overlay()

      # TODO: still needed? skip this and just fetch?
      # Lightweight internt check
      internet_ok, update_available = check_for_update()
      if internet_ok and not update_available:
        update_failed_count = 0

      # Fetch update
      if internet_ok:
        new_version = fetch_update(wait_helper)
        update_failed_count = 0
    except subprocess.CalledProcessError as e:
      cloudlog.event(
        "update process failed",
        cmd=e.cmd,
        output=e.output,
        returncode=e.returncode
      )
      exception = f"command failed: {e.cmd}\n{e.output}"
      overlay_init.unlink(missing_ok=True)
    except Exception as e:
      cloudlog.exception("uncaught updated exception, shouldn't happen")
      exception = str(e)
      overlay_init.unlink(missing_ok=True)

    if not wait_helper.shutdown:
      try:
        set_params(new_version, update_failed_count, exception)
      except Exception:
        cloudlog.exception("uncaught updated exception while setting params, shouldn't happen")

    # infrequent attempts if we successfully updated recently
    wait_helper.sleep(5*60 if update_failed_count > 0 else 90*60)

  dismount_overlay()


if __name__ == "__main__":
  main()
