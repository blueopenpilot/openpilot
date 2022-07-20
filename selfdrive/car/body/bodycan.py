def create_control(packer, torque_l, torque_r, idx):
  can_bus = 0

  values = {
    "TORQUE_L": torque_l,
    "TORQUE_R": torque_r,
  }

  return packer.make_can_msg("TORQUE_CMD", can_bus, values, idx)
