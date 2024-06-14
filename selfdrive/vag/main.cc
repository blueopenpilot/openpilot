#include <cassert>
#include <stdio.h>

#include "common/swaglog.h"
#include "common/util.h"
#include "system/hardware/hw.h"
#include "selfdrive/vag/vag.h"

int main(int argc, char *argv[]) {
  LOGW("starting vagd");

  if (!Hardware::PC()) {
    int err;
    err = util::set_realtime_priority(54);
    assert(err == 0);
    err = util::set_core_affinity({0, 1, 2, 3});
    assert(err == 0);
  }

  std::vector<std::string> serials(argv + 1, argv + argc);
  vag_main_thread(serials);

  return 0;
}
