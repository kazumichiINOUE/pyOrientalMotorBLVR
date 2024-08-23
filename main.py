"""
    Robot control program 

    author: Kazumichi INOUE
    date: 2024/08/12
"""

import serial
import time
from datetime import datetime
import math
from math import pi, cos, sin
import pygame
import sys
import cv2
import copy
import numpy as np

import Query as qry
import DummySerial

# 色指定の形式変換
def hex_to_rgb(hex_color):
    hex_color = hex_color.lstrip('#')
    return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))

"""
Urg class
"""
class Urg:
    """
    Urgクラスの定義
    """
    def __init__(self, device_file, baurate):
        self.device_file = device_file
        self.baurate = baurate
        self.timeout = 1
        try:
            self.ser = serial.Serial(self.device_file, self.baurate, timeout=self.timeout)
        except serial.SerialException as e:
            print(f"Connection error {e}", file=sys.stderr)
            print("Bye")
            sys.exit(0)
        time.sleep(2)

        # VVコマンドを送信し通信テスト
        success, response = cmd_VV(self.ser)
        if success is True:
            print("[OK] VV")
            if DEBUG_MODE:
                print(response, file=sys.stderr)
        else:
            print("[False] VV", file=sys.stderr)
            sys.exit(0)

        time.sleep(1)

        # PPコマンドを送信し通信テスト
        success, response = cmd_PP(self.ser)
        if success is True:
            print("[OK] PP")
            if DEBUG_MODE:
                print(response, file=sys.stderr)
        else:
            print("[False] PP", file=sys.stderr)
            sys.exit(0)

        time.sleep(1)

        # IIコマンドを送信し通信テスト
        success, response = cmd_II(self.ser)
        if success is True:
            print("[OK] II")
            if DEBUG_MODE:
                print(response, file=sys.stderr)
        else:
            print("[False] II", file=sys.stderr)
            sys.exit(0)

        # 通信テストを合格
        print("Connect test all clear.")

    def one_shot(self):
        """
        1回だけ計測する
        """
        # MDコマンドを送信しone-shot計測
        success, _, data = cmd_MD(self.ser)
        urg_data = []
        if success is True:
            data_strings = ""
            for p in data:
                if len(p) > 0:
                    # check_sum = p[-1]
                    body = p[:-1]
                    data_strings += body
            for i in range(0, len(data_strings), 3):
                three_chars = data_strings[i:i+3]
                binary = []
                for char in three_chars:
                    ch = ord(char) - 0x30
                    hex_ascii_value = hex(ch)
                    bin_val = bin(int(hex_ascii_value, 16))
                    # 2進数を6ビットの2進数に変換
                    binary.append(format(int(bin_val, 2), '06b'))
                combined_binary_24bit = ''.join(binary)
                if all(c in '01' for c in combined_binary_24bit):
                    r = int(combined_binary_24bit, 2)
                    index = int(i/3.0)
                    urg_data.append((index, r))
                else:
                    print("不正なデータが含まれています")
                    self.close()
                    sys.exit(0)
            return True, urg_data
        return False, urg_data

    def one_shot_intensity(self):
        """
        1回だけ計測する（反射強度付き）
        """
        # MEコマンドを送信しone-shot計測
        success, _, data = cmd_ME(self.ser)
        urg_data = []
        if success is True:
            data_strings = ""
            for p in data:
                if len(p) > 0:
                    # check_sum = p[-1]
                    body = p[:-1]
                    data_strings += body
            print(f"data_length:{len(data_strings)}")
            print(data_strings)
            print("===")
            for i in range(0, len(data_strings), 6):
                three_chars_r = data_strings[i+0:i+3]
                three_chars_i = data_strings[i+3:i+6]
                binary_r = []
                binary_i = []
                for char in three_chars_r:
                    ch = ord(char) - 0x30
                    hex_ascii_value = hex(ch)
                    bin_val = bin(int(hex_ascii_value, 16))
                    # 2進数を6ビットの2進数に変換
                    binary_r.append(format(int(bin_val, 2), '06b'))
                combined_binary_24bit_r = ''.join(binary_r)

                for char in three_chars_i:
                    ch = ord(char) - 0x30
                    hex_ascii_value = hex(ch)
                    bin_val = bin(int(hex_ascii_value, 16))
                    # 2進数を6ビットの2進数に変換
                    binary_i.append(format(int(bin_val, 2), '06b'))
                combined_binary_24bit_i = ''.join(binary_i)
                if all(c in '01' for c in combined_binary_24bit_r) or all(c in '01' for c in combined_binary_24bit_i):
                    r = int(combined_binary_24bit_r, 2)
                    intensity = int(combined_binary_24bit_i, 2)
                    index = int(i/3.0)
                    urg_data.append((index, r, intensity))
                else:
                    print("不正なデータが含まれています")
                    self.close()
                    sys.exit(0)
            return True, urg_data
        return False, urg_data

    def close(self):
        """
        close serial port
        """
        self.ser.close()

def remove_semicolon_followed_by_char(line):
    # 改行を除去する
    line = line.rstrip('\n')
    # 「;」の位置を見つける
    semicolon_index = line.find(';')
    if semicolon_index != -1:
        # 「;」の前の部分のみを抽出
        return line[:semicolon_index]
    return line

def cmd_VV(ser_dev):
    # VVコマンドを送信（デバイス情報を要求）
    ser_dev.write(b'VV\n')
    # 応答を読み取る
    ret = []
    for _ in range(8):
        response = ser_dev.read_until().decode('utf-8')
        ret.append(remove_semicolon_followed_by_char(response))
    if len(ret) > 0:
        return True, ret

    return False, ret

def cmd_PP(ser_dev):
    # PPコマンドを送信（デバイスパラメータ情報を要求）
    ser_dev.write(b'PP\n')
    # 応答を読み取る
    ret = []
    for _ in range(11):
        response = ser_dev.read_until().decode('utf-8')
        ret.append(remove_semicolon_followed_by_char(response))
    if len(ret) > 0:
        return True, ret
    return False, ret

def cmd_II(ser_dev):
    # IIコマンドを送信（ステータス情報を要求）
    ser_dev.write(b'II\n')
    # 応答を読み取る
    ret = []
    for _ in range(10):
        response = ser_dev.read_until().decode('utf-8')
        ret.append(remove_semicolon_followed_by_char(response))
    if len(ret) > 0:
        return True, ret
    return False, ret

def cmd_MD(ser_dev):
    # MDコマンドを送信（距離データの取得）
    ser_dev.write(b'MD0000108001101\n')
    #ser_dev.write(b'MD0044072501101\n') # for Classic URG
    # 応答を読み取る
    head = []
    data = []
    for _ in range(6):
        response = ser_dev.read_until().decode('utf-8')
        head.append(remove_semicolon_followed_by_char(response))
    LOOP_NUM_FOR_READ = 52  # Classic URGでは33
    for _ in range(LOOP_NUM_FOR_READ):
        response = ser_dev.read_until().decode('utf-8')
        response = response.rstrip('\n')
        data.append(response)
    if len(head) > 0:
        return True, head, data
    return False, head, []

def cmd_ME(ser_dev):
    # MDコマンドを送信（距離データの取得）
    ser_dev.write(b'ME0000108001101\n')
    #ser_dev.write(b'MD0044072501101\n') # for Classic URG
    # 応答を読み取る
    head = []
    data = []
    for _ in range(6):
        response = ser_dev.read_until().decode('utf-8')
        head.append(remove_semicolon_followed_by_char(response))
    LOOP_NUM_FOR_READ = 103  # Classic URGでは33
    for _ in range(LOOP_NUM_FOR_READ):
        response = ser_dev.read_until().decode('utf-8')
        response = response.rstrip('\n')
        data.append(response)
    if len(head) > 0:
        return True, head, data
    return False, head, []



def index2angle(index):
    return (index + 0) * 360/1440.0 - 135.0
    # 以下の数値はUBG-04LX-F01の設定値
    #return (index + 44) * 360/1024 - 135.0

def angle2index(angle):
    return int((angle - (-135.0)*1440.0/360))

def deg2rad(deg):
    return deg * pi/180.0


# Initialize LiDAR
DEBUG_MODE = False
urg = Urg('/dev/cu.usbmodem213301', 115200)

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

# Initialize Oriental motor BLV-R 
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
    
    count = 0
    count_max = 200
    center_range = []
    center_range2 = []
    time_stamp = []

    start_angle = -135.0
    end_angle   =  135.0
    step_angle  = 0.25
    echo_size   = 3

    height = 700
    width = 700
    csize = 0.0125/1
    img_org = np.zeros((height, width, 3), dtype=np.uint8)
    img_org[:] = hex_to_rgb('#e6e7ed')
    cv2.line(img_org, (0, height//2), (width, height//2), hex_to_rgb('#6c6e75'), 1)
    cv2.line(img_org, (width//2, 0),  (width//2, height), hex_to_rgb('#6c6e75'), 1)
    ticks = np.arange(-width/2*csize, width/2*csize + 0.5, 1)
    for i in ticks:
        if i == 0:
            continue
        tick_x = int(i / csize)
        cv2.line(img_org, (width//2 - tick_x, height//2-10), (width//2 - tick_x, height//2 + 10), hex_to_rgb('#6c6e75'), 1)
        cv2.line(img_org, (width//2 -10, height//2 - tick_x), (width//2 + 10, height//2- tick_x), hex_to_rgb('#6c6e75'), 1)
    img = copy.deepcopy(img_org)
    cv2.imshow("LiDAR", img)

    # Create file name to store LiDAR data
    #file_name = input("データを保存するファイル名を指示してください（終了する場合はEnterキーを押してください）: ")
    timestamp = int(time.time())
    formatted_date = datetime.fromtimestamp(timestamp).strftime('%Y_%m_%d_%H_%M_%S')
    file_name = "urglog_" + formatted_date

    if file_name == "":
        print("プログラムを終了します。")
        exit(0)
    with open(file_name, "w") as file:
        pass

    ########################################
    # Main loop start
    ########################################
    while True:
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
                cv2.rectangle(img, (ix-2, iy-2), (ix+2, iy+2), hex_to_rgb('#33635c'), -1)
        cv2.imshow("LiDAR", img)
        cv2.waitKey(5)

        # Get joystick status
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 5:
                    with open(file_name, "a") as file:
                        data_size = 1081*3
                        ts = int(time.time() * 1e3)
                        file.write(f"LASERSCANRT {ts} {data_size} {start_angle} {end_angle} {step_angle} {echo_size} ")
                        for d in urg_data:
                            file.write(f"{d[1]} 0 0 ")
                        file.write(f"{ts}\n")
                    print("Stored LiDAR data")
                    break
                elif event.button == 11:
                    qry.read_state(ser)
                    break

            # Get button state of joystick
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
        qry.simple_send_cmd(ser, Query_NET_ID_WRITE);  # print(f"send v:{v}, w:{w} to LR")

        pygame.time.wait(100)

    # Terminate process
    v = 0.0
    w = 0.0
    Query_NET_ID_WRITE = qry.calc_vw2hex(v, w)
    qry.simple_send_cmd(ser, Query_NET_ID_WRITE);   print(f"send v:{v}, w:{w} to LR")
    time.sleep(2)
    
    # turn off motor drivers
    qry.simple_send_cmd(ser, qry.Query_Write_Servo_OFF_R);   print("servo off R")
    qry.simple_send_cmd(ser, qry.Query_Write_Servo_OFF_L);   print("servo off L")
    
    urg.close()
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
    urg.close()
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
    urg.close()
    ser.close()
