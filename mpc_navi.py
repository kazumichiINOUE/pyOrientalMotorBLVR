"""
    Robot control program Using MPC

    author: Kazumichi INOUE
    date: 2024/10/28
"""

import serial
import time
from datetime import datetime
import math
from math import pi, cos, sin
import sys
import cv2
import copy
import numpy as np
import casadi

import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame

# My modules
import Query as qry
import DummySerial
import DummyLidar
import DummyJoystick
import ReadConfig as rc
from Colors import hex_to_rgb
import Odometory
import Lidar

import MotorDriver as md
import MPC

obstacles = [(3.0, 2)]
x_ref_lists = [
    casadi.DM([ 1.2, 0,  0]),
    casadi.DM([ 1.2, 1.0,  np.pi/2]),
    casadi.DM([ 0.0, 1.0,  np.pi]),
    casadi.DM([ 0.0, 0.0,  0])
]
"""
    ,
    casadi.DM([ 3, 3,  np.pi/2]),
    casadi.DM([ 0, 3,  np.pi*3/2]),
    casadi.DM([ 0, 0,  0]),
]
"""
#x_ref = casadi.DM([-3, 0,  np.pi/2])
u_ref = casadi.DM([0, 0])
K = 20
T = 2
dt = T/K
current_time = 0

x_init = casadi.DM([0, 0, 0])
initial_velocity = casadi.DM([0.0, 0.0])
X, U = [x_init], [initial_velocity]     # グラフにするための配列
t_eval = [0]

# Include default configuration
config = rc.read_config('config.lua')

# Initialize LiDAR
try:
    urg = Lidar.Urg(config.lidar.serial_port, config.lidar.baudrate, config)
except serial.SerialException as e:
    print(f"Error: {e}")
    print("Connect to Dummy Lidar Class.")
    urg = DummyLidar.DummyLidar()

# Initialize pygame & joypad
pygame.init()
pygame.joystick.init()

# 接続されているジョイスティックの数を確認
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("ジョイスティックが接続されていません")

# 最初のジョイスティックを取得
if joystick_count > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"検出されたジョイスティック: {joystick.get_name()}")
else:
    joystick = DummyJoystick.DummyJoystick()

try:
    # Initialize Oriental motor BLV-R 
    fd = md.init(config.motor.serial_port, config.motor.baudrate)
    md.turn_on_motors(fd)
    #md.free_motors(fd)
    #md.turn_off_motors(fd)

    count = 0
    count_max = 200
    center_range = []
    center_range2 = []
    time_stamp = []

    start_angle = config.lidar.start_angle
    end_angle   = config.lidar.end_angle
    step_angle  = config.lidar.step_angle
    echo_size   = config.lidar.echo_size

    height = config.map.window_height
    width = config.map.window_width
    csize = config.map.csize
    img_org = np.zeros((height, width, 3), dtype=np.uint8)
    img_org[:] = hex_to_rgb(config.map.color.bg)
    cv2.line(img_org, (0, height//2), (width, height//2), hex_to_rgb(config.map.color.axis), 1)
    cv2.line(img_org, (width//2, 0),  (width//2, height), hex_to_rgb(config.map.color.axis), 1)
    ticks = np.arange(-width/2*csize, width/2*csize + 0.5, 1)
    for i in ticks:
        if i == 0:
            continue
        tick_x = int(i / csize)
        cv2.line(img_org, (width//2 - tick_x, height//2-10), (width//2 - tick_x, height//2 + 10), hex_to_rgb(config.map.color.axis), 1)
        cv2.line(img_org, (width//2 -10, height//2 - tick_x), (width//2 + 10, height//2- tick_x), hex_to_rgb(config.map.color.axis), 1)
    img = copy.deepcopy(img_org)
    cv2.imshow("LiDAR", img)

    # Create file name to store LiDAR data
    if config.lidar.store_data:
        #file_name = input("データを保存するファイル名を指示してください（終了する場合はEnterキーを押してください）: ")
        timestamp = int(time.time())
        formatted_date = datetime.fromtimestamp(timestamp).strftime('%Y_%m_%d_%H_%M_%S')
        file_name = "urglog_" + formatted_date

        if file_name == "":
            print("プログラムを終了します。")
            exit(0)
        with open(file_name, "w") as file:
            pass
    elif config.lidar.store_data == False:
        while True:
            res = input("LiDARデータは保存されません．本当に良いですか？ [y/N]")
            if res == "" or res == "n" or res == "N":
                v = 0.0
                w = 0.0
                md.send_vw(fd, v, w)
                time.sleep(2)
                
                md.turn_off_motors(fd)
                
                urg.close()
                sys.exit()
            elif res == "y" or res == "Y":
                break

    ###
    # JOYBUTTON NUM
    ###
    NUM_JOY_GET_LIDAR  = 5
    NUM_JOY_GET_STATE  = 11
    NUM_JOY_TURN_LEFT  = 0
    NUM_JOY_GO_FORWARD = 1
    NUM_JOY_GO_BACK    = 2
    NUM_JOY_TURN_RIGHT = 3
    NUM_JOY_SHUTDOWN   = 10

    odo = Odometory.Odometory()
    ########################################
    # Main loop start
    ########################################
    while True:
        ts = int(time.time() * 1e3)
        odo.rx, odo.ry, odo.ra, odo.travel, odo.rotation = md.read_odo2(fd, odo.rx, odo.ry, odo.ra, odo.travel, odo.rotation)
        print(f"{odo.rx:.3f} {odo.ry:.3f} {odo.ra*180/math.pi:.1f} {odo.travel:.3f} {odo.rotation*180/math.pi:.1f}")
        #with open("enclog", "a") as file:
        #    file.write(f"{ts} {odo.rx} {odo.ry} {odo.ra} {odo.travel} {az}\n")
        for x_ref in x_ref_lists:
            # MPC Controller インスタンス作成
            mpc = MPC.MPCController(x_ref, u_ref, K, T, obstacles = obstacles)
            x0 = casadi.DM.zeros(mpc.total_vars)
            x_current = x_init
            prev_x = x_current
            loop_count = 0
            while True:
                current_time += dt
                if casadi.sqrt((x_ref[0] - x_current[0])**2 + (x_ref[1] - x_current[1])**2) < 0.2:
                    x_init = x_current
                    break
                u_opt, x0 = mpc.compute_optimal_control(x_current, x0)
                md.send_vw(fd, u_opt[0], u_opt[1])
                #time.sleep(dt)
                odo.rx, odo.ry, odo.ra, odo.travel, odo.rotation = md.read_odo2(fd, odo.rx, odo.ry, odo.ra, odo.travel, odo.rotation)
                print(x_ref, x_current)
                print(f"{odo.rx:.3f} {odo.ry:.3f} {odo.ra*180/math.pi:.1f} {odo.travel:.3f} {odo.rotation*180/math.pi:.1f}")
                x_current[0] = odo.rx
                x_current[1] = odo.ry
                x_current[2] = odo.ra
                #x_current = mpc.I(x0=x_current, p=u_opt)["xf"]
                #X.append(x_current)
                #U.append(u_opt)
                #t_eval.append(current_time)
                if casadi.sumsqr(prev_x - x_current) < 1e16:
                    loop_count += 1
                    if loop_count > 200:
                        loop_count = 0
                        x_init = x_current
                        break
                prev_x = x_current

        # Terminate process
        v = 0.0
        w = 0.0
        md.send_vw(fd, v, w)
        time.sleep(2)
        md.turn_off_motors(fd)
        urg.close()
        exit()
        """
        # Get LiDAR data
        success, urg_data = urg.one_shot()
        x = []
        y = []
        for index, d in enumerate(urg_data):
            angle = (index * step_angle + start_angle) * pi/180
            xd = d[1] * cos(angle) / 1000
            yd = d[1] * sin(angle) / 1000
            x.append(xd)
            y.append(yd)

        # Show LiDAR data
        img = copy.deepcopy(img_org)
        for px, py in zip(x, y):
            tmp_px = -py
            py = px
            px = tmp_px
            ix =int( px / csize + width//2)
            iy =int(-py / csize + height//2)
            if ix >= 0 and ix < width and iy >= 0 and iy < height:
                cv2.rectangle(img, (ix-2, iy-2), (ix+2, iy+2), hex_to_rgb(config.map.color.point), -1)
        cv2.imshow("LiDAR", img)
        cv2.waitKey(5)

        # Get joystick status
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == NUM_JOY_GET_LIDAR:
                    ts = int(time.time() * 1e3)
                    if config.lidar.store_data:
                        with open(file_name, "a") as file:
                            data_size = 1081*3
                            file.write(f"LASERSCANRT {ts} {data_size} {start_angle} {end_angle} {step_angle} {echo_size} ")
                            for d in urg_data:
                                file.write(f"{d[1]} 0 0 ")
                            file.write(f"{ts}\n")
                        print("Stored LiDAR data")

                    break
                elif event.button == NUM_JOY_GET_STATE:
                    #print(event.button)
                    md.show_state(fd)
                    #qry.read_state(ser)
                    break

            # Get button state of joystick
            button_pressed_turn_left  = joystick.get_button(NUM_JOY_TURN_LEFT)  
            button_pressed_go_forward = joystick.get_button(NUM_JOY_GO_FORWARD)  
            button_pressed_go_back    = joystick.get_button(NUM_JOY_GO_BACK) 
            button_pressed_turn_right = joystick.get_button(NUM_JOY_TURN_RIGHT) 
            button_pressed_shutdown   = joystick.get_button(NUM_JOY_SHUTDOWN)

        v = 0.0
        w = 0.0
        if button_pressed_turn_left: # Turn Left
            v = 0.0
            w = math.pi/4
            #print("0ボタンが押されています", v, w)
        elif button_pressed_go_forward: # Go forward
            v = 0.2
            w = 0.0
            #print("1ボタンが押されています", v, w)
        elif button_pressed_go_back: # Go back
            v =-0.2
            w = 0.0
            #print("2ボタンが押されています", v, w)
        elif button_pressed_turn_right: # Turn Right
            v = 0.0
            w = -math.pi/4
            #print("3ボタンが押されています", v, w)
        elif button_pressed_shutdown: # Shutdown
            print("Pressed Stop button")
            break
        md.send_vw(fd, v, w)

        pygame.time.wait(10)
        """

    # Terminate process
    v = 0.0
    w = 0.0
    md.send_vw(fd, v, w)
    time.sleep(2)
    
    md.turn_off_motors(fd)
    
    urg.close()

except KeyboardInterrupt:
    print("Pressed Ctrl + C")
    print("Shutting down now...")
    # Ctrl+Cが押されたときに安全に終了する
    v = 0.0
    w = 0.0
    md.send_vw(fd, v, w)
    time.sleep(2)
    md.turn_off_motors(fd)
    print("Closing serial connection.")
    urg.close()

except Exception as e:
    # その他のエラーが発生した場合
    print(f"An error occurred: {e}")
    v = 0.0
    w = 0.0
    md.send_vw(fd, v, w)
    time.sleep(2)
    md.turn_off_motors(fd)
    print("Closing serial connection.")
    urg.close()
