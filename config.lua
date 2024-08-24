print("Hello, coyomi config.lua")

local config = {
  robot = {
    robot_name = "coyomi",
    version = "3.0.0",
    debug_mode = true
  },

  lidar = {
    debug_mode  = false,
    store_data  = false,
    serial_port = '/dev/cu.usbmodem213301',
    baudrate    = 115200,
    start_angle = -135.0,
    end_angle   =  135.0,
    step_angle  = 0.25,
    echo_size   = 3,
  },

  map = {
    csize = 0.0125,

    height = 700,
    width  = 700,

    color = {
      bg    = '#e6e7ed',
      axis  = '#6c6e75',
      point = '#33635c',
    },
  },
}

return config
