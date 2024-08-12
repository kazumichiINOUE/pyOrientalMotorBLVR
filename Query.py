"""
    ID=2 に終端抵抗の設定をしてください
    設定はOriental Motor 提供のサポートソフトMEXE02を利用
"""

### 車体・モーターのパラメータ
WHEEL_D = 0.314     # 8inch
WHEEL_T = 0.4274
M_PI = 3.1415
GEAR_RATIO = 50

### 受信メッセージ受取用
MAX_BUFFER_SIZE = 512

# クエリデータ
Query_IDshare_L = bytearray([
    0x01,                    # ID=1
	0x10, 
	0x09, 0x80, 
	0x00, 0x06, 
	0x0C,
	0x00, 0x00, 0x00, 0x0F, 
	0x00, 0x00, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x01,
	0x00, 0x00
])

Query_IDshare_R = bytearray([
    0x02,                    # ID=2
	0x10,
	0x09, 0x80, 
	0x00, 0x06, 
	0x0C, 
	0x00, 0x00, 0x00, 0x0F, 
	0x00, 0x00, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x02, 
	0x00, 0x00
])

Query_READ_L = bytearray([
    0x01,                    # ID=1
	0x10, 
	0x09, 0x90, 
	0x00, 0x0C, 
	0x18, 
	0x00, 0x00, 0x00, 0x40, 
	0x00, 0x00, 0x00, 0x7C,
    0x00, 0x00, 0x00, 0x7D, 
	0x00, 0x00, 0x00, 0x66, 
	0x00, 0x00, 0x00, 0x9E, 
	0x00, 0x00, 0x00, 0xA4, 
	0x00, 0x00
])

Query_READ_R = bytearray([
    0x02,                    # ID=2
	0x10, 
	0x09, 0x90, 
	0x00, 0x0C, 
	0x18, 
	0x00, 0x00, 0x00, 0x40, 
	0x00, 0x00, 0x00, 0x7C,
    0x00, 0x00, 0x00, 0x7D, 
	0x00, 0x00, 0x00, 0x66, 
	0x00, 0x00, 0x00, 0x9E, 
	0x00, 0x00, 0x00, 0xA4, 
	0x00, 0x00
])

Query_WRITE_L = bytearray([
    0x01,                    # ID=1
	0x10, 
	0x09, 0xA8, 
	0x00, 0x0C, 
	0x18, 
	0x00, 0x00, 0x00, 0x2D, 
	0x00, 0x00, 0x00, 0x2E,
    0x00, 0x00, 0x00, 0x2F, 
	0x00, 0x00, 0x00, 0x30, 
	0x00, 0x00, 0x00, 0x31, 
	0x00, 0x00, 0x00, 0x33, 
	0x00, 0x00
])

Query_WRITE_R = bytearray([
    0x02,                    # ID=2
	0x10, 
	0x09, 0xA8, 
	0x00, 0x0C, 
	0x18, 
	0x00, 0x00, 0x00, 0x2D, 
	0x00, 0x00, 0x00, 0x2E,
    0x00, 0x00, 0x00, 0x2F, 
	0x00, 0x00, 0x00, 0x30, 
	0x00, 0x00, 0x00, 0x31, 
	0x00, 0x00, 0x00, 0x33, 
	0x00, 0x00
])

Query_Write_Servo_ON_L = bytearray([
    0x01,                    # ID=1
	0x10, 
	0x00, 0x7C, 
	0x00, 0x02, 
	0x04, 
	0x00, 0x00, 0x00, 0x01, 
	0x00, 0x00
])

Query_Write_Servo_ON_R = bytearray([
    0x02,                    # ID=2
	0x10, 
	0x00, 0x7C, 
	0x00, 0x02, 
	0x04, 
	0x00, 0x00, 0x00, 0x01, 
	0x00, 0x00
])

Query_Write_Servo_OFF_L = bytearray([
    0x01,                    # ID=1
	0x10, 
	0x00, 0x7C, 
	0x00, 0x02, 
	0x04, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00
])

Query_Write_Servo_OFF_R = bytearray([
    0x02,                    # ID=2
	0x10, 
	0x00, 0x7C, 
	0x00, 0x02, 
	0x04, 
	0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00
])

Query_Write_FREE_L = bytearray([
  0x01,                    # ID=1
  0x10,       
  0x00, 0x7C,
  0x00, 0x02,
  0x04,
  0x00, 0x00, 0x00, 0x41,  # FREE
  0x00, 0x00
])

Query_Write_FREE_R = bytearray([
  0x02,                    # ID=2
  0x10,             
  0x00, 0x7C,
  0x00, 0x02,
  0x04,
  0x00, 0x00, 0x00, 0x41,  # FREE
  0x00, 0x00
])

Query_NET_ID_READ = bytearray([
  0x0F, 
  0x03, 
  0x00, 0x00,
  0x00, 0x1A,
  0x00, 0x00
])

def calc_vw2hex(v, w):
    """
        (v, w)　速度・角速度指令をクエリデータに換算する
        ID Share モードを使用する
    """
    wr = v/(WHEEL_D/2) + w*WHEEL_T/WHEEL_D;
    wl = v/(WHEEL_D/2) - w*WHEEL_T/WHEEL_D;
    motor_wr_rpm = -wr/2/M_PI * GEAR_RATIO * 60;
    motor_wl_rpm =  wl/2/M_PI * GEAR_RATIO * 60;
    Query_NET_ID_WRITE = bytearray([
        0x0F, 
        0x10,
        0x00, 0x00,
        0x00, 0x18,
        0x30,
        0x00, 0x00, 0x00, 0x10,   # 01:drive mode
        0x00, 0x00, 0x00, 0x00,   #    position
        0x00, 0x00, 0x00, 0x00,   #    speed
        0x00, 0x00, 0x07, 0xD0,   #    Accell rate
        0x00, 0x00, 0x03, 0xE8,   #    Decell rate
        0x00, 0x00, 0x00, 0x01,   #    Trigger
        0x00, 0x00, 0x00, 0x10,   # 02:drive mode
        0x00, 0x00, 0x00, 0x00,   #    position
        0x00, 0x00, 0x00, 0x00,   #    speed
        0x00, 0x00, 0x07, 0xD0,   #    Accell rate
        0x00, 0x00, 0x03, 0xE8,   #    Decell rate
        0x00, 0x00, 0x00, 0x01,   #    Trigger
        0x00, 0x00
    ])
    Query_NET_ID_WRITE[15] = (int(motor_wr_rpm) >> 24) & 0xFF
    Query_NET_ID_WRITE[16] = (int(motor_wr_rpm) >> 16) & 0xFF
    Query_NET_ID_WRITE[17] = (int(motor_wr_rpm) >> 8) & 0xFF
    Query_NET_ID_WRITE[18] = int(motor_wr_rpm) & 0xFF
    Query_NET_ID_WRITE[39] = (int(motor_wl_rpm) >> 24) & 0xFF
    Query_NET_ID_WRITE[40] = (int(motor_wl_rpm) >> 16) & 0xFF
    Query_NET_ID_WRITE[41] = (int(motor_wl_rpm) >> 8) & 0xFF
    Query_NET_ID_WRITE[42] = int(motor_wl_rpm) & 0xFF
    return Query_NET_ID_WRITE

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

'''
def read_state():
    buf = bytearray(MAX_BUFFER_SIZE)
    
    send_cmd(Query_NET_ID_READ, len(Query_NET_ID_READ))
    read_res(buf, 57)
    
    OFFSET = 26
    alarm_code_R = int.from_bytes(buf[3:7], 'big')
    temp_driver_R = int.from_bytes(buf[7:11], 'big') * 0.1
    temp_motor_R = int.from_bytes(buf[11:15], 'big') * 0.1
    position_R = int.from_bytes(buf[15:19], 'big')
    power_R = int.from_bytes(buf[19:23], 'big')
    voltage_R = int.from_bytes(buf[23:27], 'big') * 0.1
    
    alarm_code_L = int.from_bytes(buf[3 + OFFSET:7 + OFFSET], 'big')
    temp_driver_L = int.from_bytes(buf[7 + OFFSET:11 + OFFSET], 'big') * 0.1
    temp_motor_L = int.from_bytes(buf[11 + OFFSET:15 + OFFSET], 'big') * 0.1
    position_L = int.from_bytes(buf[15 + OFFSET:19 + OFFSET], 'big')
    power_L = int.from_bytes(buf[19 + OFFSET:23 + OFFSET], 'big')
    voltage_L = int.from_bytes(buf[23 + OFFSET:27 + OFFSET], 'big') * 0.1

    STEP_RESOLUTION = 1  # Placeholder value, replace with actual
    WHEEL_D = 1  # Placeholder value, replace with actual
    GEAR_RATIO = 1  # Placeholder value, replace with actual
    WHEEL_T = 1  # Placeholder value, replace with actual

    dist_L = position_L * STEP_RESOLUTION * 0.5 * WHEEL_D / GEAR_RATIO
    dist_R = position_R * STEP_RESOLUTION * 0.5 * WHEEL_D / GEAR_RATIO
    travel = (dist_L + dist_R) / 2.0
    rotation = (dist_R - dist_L) / WHEEL_T
    voltage = (voltage_L + voltage_R) / 2.0

    # Debug prints (optional)
    print(f"Alarm Code R: {alarm_code_R}")
    print(f"Temp Driver R: {temp_driver_R}")
    print(f"Temp Motor R: {temp_motor_R}")
    print(f"Position R: {position_R}")
    print(f"Power R: {power_R}")
    print(f"Voltage R: {voltage_R}")

    print(f"Alarm Code L: {alarm_code_L}")
    print(f"Temp Driver L: {temp_driver_L}")
    print(f"Temp Motor L: {temp_motor_L}")
    print(f"Position L: {position_L}")
    print(f"Power L: {power_L}")
    print(f"Voltage L: {voltage_L}")

    print(f"Dist L: {dist_L}")
    print(f"Dist R: {dist_R}")
    print(f"Travel: {travel}")
    print(f"Rotation: {rotation}")
    print(f"Voltage: {voltage}")
'''
