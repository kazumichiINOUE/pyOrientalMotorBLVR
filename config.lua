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
    serial_port = '/dev/cu.usbmodem113301',
    baudrate    = 115200,
    start_angle = -135.0,
    end_angle   =  135.0,
    step_angle  = 0.25,
    echo_size   = 3,
  },

  map = {
    csize = 0.025,

    window_height = 500,
    window_width  = 500,

    color = {
      bg    = '#e6e7ed',
      axis  = '#6c6e75',
      point = '#33635c',
    },
  },

  motor = {
    debug_mode  = false,
    wheel_d = 0.314, -- 8inch
    wheel_t = 0.4274,
    step_resolution = 0.01/180*math.pi,
    gear_ratio = 50.0,
  },

  slam = {
    window_height = 1000,
    window_width = 2000,
    origin_x = 1000,
    origin_y = 500,
    csize = 0.025, --[m]
    scan_data_size = 1081,
    skip = 32,
    -- offline SLAM settings as follows:
    urg_log_file = "./urglog_2024_08_27_17_20_20",
    result_path = "./result/path",
    result_gmap = "./result/result.png",
  },
}

return config
