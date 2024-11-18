"""
    Robot control program using joypad with MPC

    author: Kazumichi INOUE
    date: 2024/10/30
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

def draw_lidar_on_img(img_org, urg_data):
    d_values = np.array([d[1] for d in urg_data])  # LiDARデータの距離成分を抽出
    xd = d_values * cs / 1000  # X座標変換
    yd = d_values * sn / 1000  # Y座標変換
    ix = (-yd / csize + width // 2).astype(int)
    iy = (-xd / csize + height // 2).astype(int)
    # 範囲内の座標をフィルタリング
    valid_mask = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    ix = ix[valid_mask]
    iy = iy[valid_mask]
    # 描画
    img = copy.deepcopy(img_org)
    for x, y in zip(ix, iy):
        img[y-2:y+3, x-2:x+3] = color  # 矩形領域を一括で塗りつぶす
    return img

try:
    # Initialize Oriental motor BLV-R 
    fd = md.init(config.motor.serial_port, config.motor.baudrate)
    md.turn_on_motors(fd)
    #md.free_motors(fd)
    #md.turn_off_motors(fd)


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
            print("LiDARデータは保存されません")
            res = "y"
            #res = input("LiDARデータは保存されません．本当に良いですか？ [y/N]")
            if res == "" or res == "n" or res == "N":
                v = 0.0
                w = 0.0
                md.send_vw(fd, v, w)
                #Query_NET_ID_WRITE = qry.calc_vw2hex(v, w)
                #qry.simple_send_cmd(ser, Query_NET_ID_WRITE);   print(f"send v:{v}, w:{w} to LR")
                time.sleep(2)
                
                md.turn_off_motors(fd)
                
                urg.close()
                #ser.close()
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

    H_normalized = np.loadtxt("./Hmatrix.txt")
    H_inv = np.linalg.inv(H_normalized)
    H_inv = H_inv/H_inv[2][2]
    ## 中心(1500, 0)の半径500の円上の点のリスト
    center_x = 0
    center_y = 0
    point_on_circle = [(center_x + 50 * cos(i * pi / 180), center_y + 50 * sin(i * pi / 180)) for i in range(360)]
    cap = cv2.VideoCapture(0)  # '0' は内蔵カメラ
    # LiDAR変換用にcos, sin のリストを作る
    cs = [cos((i * 0.25 - 135.0)*pi/180) for i in range(1081)]
    sn = [sin((i * 0.25 - 135.0)*pi/180) for i in range(1081)]
    # 色をNumPyで表現
    color = np.array(hex_to_rgb(config.map.color.point), dtype=np.uint8)
    ########################################
    # Main loop start
    ########################################
    while True:
        ts = int(time.time() * 1e3)

        ret, frame = cap.read()
        point_on_circle = [(center_x + 50 * cos(i * pi / 180), center_y + 50 * sin(i * pi / 180)) for i in range(360)]
        for tx, ty in point_on_circle:
            p1 = np.array([tx, ty, 1])
            p_origin = np.dot(H_inv, p1)
            p_origin = p_origin/p_origin[2]
            cv2.circle(frame, (int(p_origin[0]), int(p_origin[1])), 2, (0, 0, 255), -1)
        cv2.imshow('Capture image', frame)

        # Get & Show LiDAR data
        success, urg_data = urg.one_shot()
        if success:
            img = draw_lidar_on_img(img_org, urg_data)

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
                    print(event.button)
                    #qry.read_state(ser)
                    break

            # Get button state of joystick
            button_pressed_turn_left  = joystick.get_button(NUM_JOY_TURN_LEFT)  
            button_pressed_go_forward = joystick.get_button(NUM_JOY_GO_FORWARD)  
            button_pressed_go_back    = joystick.get_button(NUM_JOY_GO_BACK) 
            button_pressed_turn_right = joystick.get_button(NUM_JOY_TURN_RIGHT) 
            button_pressed_shutdown   = joystick.get_button(NUM_JOY_SHUTDOWN)

        if button_pressed_turn_left: # Turn Left
            target_r = 1.0
            target_a = pi/2
            robot_a = pi/2
            center_y += 100
            #print("0ボタンが押されています")
        elif button_pressed_go_forward: # Go forward
            target_r = 1.0
            target_a = 0.0
            robot_a = 0.0
            center_x += 100
            #print("1ボタンが押されています")
        elif button_pressed_go_back: # Go back
            target_r = 1.0
            target_a = pi
            robot_a = 0.0
            center_x -= 100
            #print("2ボタンが押されています")
        elif button_pressed_turn_right: # Turn Right
            target_r = 1.0
            target_a = -pi/2
            robot_a = -pi/2
            center_y -= 100
            #print("3ボタンが押されています")
        elif button_pressed_shutdown: # Shutdown
            print("Pressed Stop button")
            break
        else:
            # joypadの入力がない場合は，現在位置を目標値点としたmpcを行う
            target_r = 0.0
            target_a = 0.0
            robot_a = 0.0

        # Get control command from MPC
        x_ref = casadi.DM([target_r*cos(target_a), target_r*sin(target_a),  robot_a])
        u_ref = casadi.DM([0, 0])
        K = 20
        T = 1
        x_init = casadi.DM([0, 0, 0])   # 常に現在のロボット座標系からスタートする
        mpc = MPC.MPCController(x_ref, u_ref, K, T)
        x0 = casadi.DM.zeros(mpc.total_vars)
        x_current = x_init
        u_opt, x0 = mpc.compute_optimal_control(x_current, x0)
        #md.send_vw(fd, u_opt[0], u_opt[1])

        tx =  int(-x_ref[1, 0]/csize) + width//2
        ty = -int( x_ref[0, 0]/csize) + height//2
        cx =  int(-center_y/1000/csize) + width//2
        cy = -int( center_x/1000/csize) + height//2
        cv2.circle(img, (tx, ty), int(1/config.map.csize/2), hex_to_rgb(config.map.color.target), 3)
        cv2.circle(img, (cx, cy), 50, (0, 0, 255), 1)
        cv2.imshow("LiDAR", img)
        cv2.waitKey(5)

        pygame.time.wait(10)

    # Terminate process
    v = 0.0
    w = 0.0
    md.send_vw(fd, v, w)
    time.sleep(2)
    
    # turn off motor drivers
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
    # turn off motor drivers
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
    # turn off motor drivers
    md.turn_off_motors(fd)
    print("Closing serial connection.")
    urg.close()
