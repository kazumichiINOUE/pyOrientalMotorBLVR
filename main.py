import serial
import time
import math

import Query as qry
import DummySerial

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
    # Start demonstration
    ########################################
    for _ in range(2):
        # 前進
        v = 0.0
        w = 0
        Query_NET_ID_WRITE = qry.calc_vw2hex(v, w)
        qry.simple_send_cmd(ser, Query_NET_ID_WRITE);   print(f"send v:{v}, w:{w} to LR")
        time.sleep(2)
        
        # 右旋回
        v = 0
        w = -math.pi/2/2
        Query_NET_ID_WRITE = qry.calc_vw2hex(v, w)
        qry.simple_send_cmd(ser, Query_NET_ID_WRITE);   print(f"send v:{v}, w:{w} to LR")
        time.sleep(2)
    
        # 前進
        v = 0.0
        w = 0
        Query_NET_ID_WRITE = qry.calc_vw2hex(v, w)
        qry.simple_send_cmd(ser, Query_NET_ID_WRITE);   print(f"send v:{v}, w:{w} to LR")
        time.sleep(2)
        
        # 左旋回
        v = 0
        w = math.pi/2/2
        Query_NET_ID_WRITE = qry.calc_vw2hex(v, w)
        qry.simple_send_cmd(ser, Query_NET_ID_WRITE);   print(f"send v:{v}, w:{w} to LR")
        time.sleep(2)
    
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
