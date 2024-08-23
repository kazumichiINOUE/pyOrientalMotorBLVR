"""
    Robot control program 

    author: Kazumichi INOUE
    date: 2024/08/12
"""

import serial
import time
import math
import pygame
import sys

import Query as qry
import DummySerial

# Initialize pygame & joypad
pygame.init()
pygame.joystick.init()

# 接続されているジョイスティックの数を確認
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("ジョイスティックが接続されていません")
    sys.exit()

# 最初のジョイスティックを取得
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"検出されたジョイスティック: {joystick.get_name()}")

# シリアルポートの設定
SERIAL_PORT = "/dev/cu.usbserial-AQ034S3S"
SERIAL_BAUDRATE = 230400    # BLV-R Default Setting
try:
    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=SERIAL_BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_EVEN,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1
    )
    ser.inter_byte_timeout = None  # INLCR, ICRNL を無効化 (デフォルトで設定されていない)
except serial.SerialException as e:
    print(f"Error: {e}")
    print("Connect to Dummy Serial Class.")
    ser = DummySerial.DummySerial()

try:
    # Send setup command to motor drivers
    qry.simple_send_cmd(ser, qry.Query_IDshare_R);      print("send id share to R")
    qry.simple_send_cmd(ser, qry.Query_IDshare_L);      print("send id share to L")
    qry.simple_send_cmd(ser, qry.Query_READ_R);         print("send read R")
    qry.simple_send_cmd(ser, qry.Query_READ_L);         print("send read L")
    qry.simple_send_cmd(ser, qry.Query_WRITE_R);        print("send write R")
    qry.simple_send_cmd(ser, qry.Query_WRITE_L);        print("send write L")
    qry.simple_send_cmd(ser, qry.Query_Write_Servo_ON_R);    print("servo on R")
    qry.simple_send_cmd(ser, qry.Query_Write_Servo_ON_L);    print("servo on L")
    
    ########################################
    # Joystick control
    ########################################
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 5:
                    print("Stored LiDAR data")
                    break
                elif event.button == 11:
                    qry.read_state(ser)
                    break

            # ボタンの状態を取得
            button_pressed_0 = joystick.get_button(0)  
            button_pressed_1 = joystick.get_button(1)  
            button_pressed_2 = joystick.get_button(2) 
            button_pressed_3 = joystick.get_button(3) 
            button_pressed_5 = joystick.get_button(5)
            button_pressed_10 = joystick.get_button(10)
            button_pressed_11 = joystick.get_button(11)

        v = 0.0
        w = 0.0
        if button_pressed_0: # Turn Left
            v = 0.0
            w = math.pi/4
            #print("0ボタンが押されています")
        elif button_pressed_1: # Go forward
            v = 0.2
            w = 0.0
            #print("1ボタンが押されています")
        elif button_pressed_2: # Go back
            v =-0.2
            w = 0.0
            #print("2ボタンが押されています")
        elif button_pressed_3: # Turn Right
            v = 0.0
            w = -math.pi/4
            #print("3ボタンが押されています")
        elif button_pressed_10: # Shutdown
            print("Pressed Stop button")
            break
        Query_NET_ID_WRITE = qry.calc_vw2hex(v, w)
        qry.simple_send_cmd(ser, Query_NET_ID_WRITE);   print(f"send v:{v}, w:{w} to LR")

        pygame.time.wait(100)

    # 停止処理
    v = 0.0
    w = 0.0
    Query_NET_ID_WRITE = qry.calc_vw2hex(v, w)
    qry.simple_send_cmd(ser, Query_NET_ID_WRITE);   print(f"send v:{v}, w:{w} to LR")
    time.sleep(2)
    
    # turn off motor drivers
    qry.simple_send_cmd(ser, qry.Query_Write_Servo_OFF_R);   print("servo off R")
    qry.simple_send_cmd(ser, qry.Query_Write_Servo_OFF_L);   print("servo off L")
    
    ser.close()

except KeyboardInterrupt:
    print("Pressed Ctrl + C")
    print("Shutting down now...")
    # Ctrl+Cが押されたときに安全に終了する
    v = 0.0
    w = 0.0
    Query_NET_ID_WRITE = qry.calc_vw2hex(v, w)
    qry.simple_send_cmd(ser, Query_NET_ID_WRITE);   print(f"send v:{v}, w:{w} to LR")
    time.sleep(2)
    # turn off motor drivers
    qry.simple_send_cmd(ser, qry.Query_Write_Servo_OFF_R);   print("servo off R")
    qry.simple_send_cmd(ser, qry.Query_Write_Servo_OFF_L);   print("servo off L")
    print("Closing serial connection.")
    ser.close()

except Exception as e:
    # その他のエラーが発生した場合
    print(f"An error occurred: {e}")
    v = 0.0
    w = 0.0
    Query_NET_ID_WRITE = qry.calc_vw2hex(v, w)
    qry.simple_send_cmd(ser, Query_NET_ID_WRITE);   print(f"send v:{v}, w:{w} to LR")
    time.sleep(2)
    # turn off motor drivers
    qry.simple_send_cmd(ser, qry.Query_Write_Servo_OFF_R);   print("servo off R")
    qry.simple_send_cmd(ser, qry.Query_Write_Servo_OFF_L);   print("servo off L")
    print("Closing serial connection.")
    ser.close()
