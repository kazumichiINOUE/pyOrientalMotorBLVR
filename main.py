import serial
import time
import math

import Query as qry

DEBUG_MODE = False

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
    exit(0)

# CRC計算関数
def calc_crc(data):
    crc = 0xFFFF
    for pos in range(len(data) - 2):
        crc ^= data[pos]
        for _ in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    data[-2] = crc & 0xFF
    data[-1] = (crc >> 8) & 0xFF

# クエリにCRCを計算して追加
def add_crc(query):
    calc_crc(query)
    return query

# コマンド送信関数
def send_cmd(ser, cmd):
    add_crc(cmd)
    ser.write(cmd)
    if DEBUG_MODE:
        print(f"[SEND] {' '.join(f'{byte:02x}' for byte in cmd)}")

# 応答を読み取る関数
def read_res(ser, length):
    SERIAL_INTERVAL_RESP = 0.1
    buf2 = bytearray()
    tries = 3
    while tries:
        buf = ser.read(1)
        if buf:
            buf2.append(buf[0])
            if len(buf2) >= length:
                break
        else:
            tries -= 1
        time.sleep(SERIAL_INTERVAL_RESP) 
    if len(buf2) < length:
        raise TimeoutError("Failed to read the expected number of bytes from the serial port.")
    return buf2

def simple_send_cmd(ser, cmd):
    send_cmd(ser, cmd)
    response = read_res(ser, 8)
    if DEBUG_MODE:
        print(f"[RESPONSE] {' '.join(f'{byte:02x}' for byte in response)}")

# Send setup command to motor drivers
simple_send_cmd(ser, qry.Query_IDshare_R);      print("send id share to R")
simple_send_cmd(ser, qry.Query_IDshare_L);      print("send id share to L")
simple_send_cmd(ser, qry.Query_READ_R);         print("send read R")
simple_send_cmd(ser, qry.Query_READ_L);         print("send read L")
simple_send_cmd(ser, qry.Query_WRITE_R);        print("send write R")
simple_send_cmd(ser, qry.Query_WRITE_L);        print("send write L")
simple_send_cmd(ser, qry.Query_Write_Servo_ON_R);    print("servo on R")
simple_send_cmd(ser, qry.Query_Write_Servo_ON_L);    print("servo on L")

########################################
# Start demonstration
########################################
try:
    for _ in range(2):
        # 前進
        v = 0.0
        w = 0
        Query_NET_ID_WRITE = qry.calc_vw2hex(v, w)
        simple_send_cmd(ser, Query_NET_ID_WRITE);   print(f"send v:{v}, w:{w} to LR")
        time.sleep(2)
        
        # 右旋回
        v = 0
        w = -math.pi/2/2
        Query_NET_ID_WRITE = qry.calc_vw2hex(v, w)
        simple_send_cmd(ser, Query_NET_ID_WRITE);   print(f"send v:{v}, w:{w} to LR")
        time.sleep(2)
    
        # 前進
        v = 0.0
        w = 0
        Query_NET_ID_WRITE = qry.calc_vw2hex(v, w)
        simple_send_cmd(ser, Query_NET_ID_WRITE);   print(f"send v:{v}, w:{w} to LR")
        time.sleep(2)
        
        # 左旋回
        v = 0
        w = math.pi/2/2
        Query_NET_ID_WRITE = qry.calc_vw2hex(v, w)
        simple_send_cmd(ser, Query_NET_ID_WRITE);   print(f"send v:{v}, w:{w} to LR")
        time.sleep(2)
    
    # 停止処理
    v = 0.0
    w = 0.0
    Query_NET_ID_WRITE = qry.calc_vw2hex(v, w)
    simple_send_cmd(ser, Query_NET_ID_WRITE);   print(f"send v:{v}, w:{w} to LR")
    time.sleep(2)
    
    # turn off motor drivers
    simple_send_cmd(ser, qry.Query_Write_Servo_OFF_R);   print("servo off R")
    simple_send_cmd(ser, qry.Query_Write_Servo_OFF_L);   print("servo off L")
    
    ser.close()

except KeyboardInterrupt:
    # Ctrl+Cが押されたときに安全に終了する
    v = 0.0
    w = 0.0
    Query_NET_ID_WRITE = qry.calc_vw2hex(v, w)
    simple_send_cmd(ser, Query_NET_ID_WRITE);   print(f"send v:{v}, w:{w} to LR")
    time.sleep(2)
    # turn off motor drivers
    simple_send_cmd(ser, qry.Query_Write_Servo_OFF_R);   print("servo off R")
    simple_send_cmd(ser, qry.Query_Write_Servo_OFF_L);   print("servo off L")
    print("Closing serial connection.")
    ser.close()

except Exception as e:
    # その他のエラーが発生した場合
    print(f"An error occurred: {e}")
    v = 0.0
    w = 0.0
    Query_NET_ID_WRITE = qry.calc_vw2hex(v, w)
    simple_send_cmd(ser, Query_NET_ID_WRITE);   print(f"send v:{v}, w:{w} to LR")
    time.sleep(2)
    # turn off motor drivers
    simple_send_cmd(ser, qry.Query_Write_Servo_OFF_R);   print("servo off R")
    simple_send_cmd(ser, qry.Query_Write_Servo_OFF_L);   print("servo off L")
    print("Closing serial connection.")
    ser.close()
