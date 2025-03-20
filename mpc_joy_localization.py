"""
    Robot control program using joypad with MPC
    Localization

    author: Kazumichi INOUE
    date: 2025/3/18
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
import casadi       # モデル予測制御
import traceback    # エラーメッセージ情報取得
import threading    # マルチスレッド
from scipy.optimize import differential_evolution, minimize

import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame

# My modules
import DummySerial      # 接続に失敗した時のためのダミー
import DummyLidar       # 同上
import DummyJoystick    # 同上
import ReadConfig as rc
from Colors import hex_to_rgb
import Odometory
import Lidar
import lidar_conversion
import lidar_draw

import MotorDriver as md
import MPC

NAVI = False # enable navigation

# Include default configuration
config = rc.read_config('config.lua')
DIR_NAME = "slam_test/250314-3"
mapInfo = rc.read_mapInfo(f"{DIR_NAME}/mapInfo.lua")

# Initialize LiDAR
try:
    urg   = Lidar.Urg(config.lidar.serial_port, config.lidar.baudrate, config)
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

def draw_lidar_on_img(img_org, urg_data, cs, sn):
    d_values = np.array([d[1] for d in urg_data])  # LiDARデータの距離成分を抽出
    img = copy.deepcopy(img_org)
    try:
        xd = d_values * cs / 1000  # X座標変換
        yd = d_values * sn / 1000  # Y座標変換
        ix = (-yd / csize + width // 2).astype(int)
        iy = (-xd / csize + height // 2).astype(int)
        # 範囲内の座標をフィルタリング
        valid_mask = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
        ix = ix[valid_mask]
        iy = iy[valid_mask]
        # 描画
        for x, y in zip(ix, iy):
            img[y-2:y+3, x-2:x+3] = color  # 矩形領域を一括で塗りつぶす
    except:
        print("d_valueにエラーの可能性")
        print(d_values)
    return img, d_values

def make_img_org(height, width, color_bg, color_axis, csize):
    img_org = np.zeros((height, width, 3), dtype=np.uint8)
    img_org[:] = hex_to_rgb(color_bg)
    cv2.line(img_org, (0, height//2), (width, height//2), hex_to_rgb(color_axis), 1)
    cv2.line(img_org, (width//2, 0),  (width//2, height), hex_to_rgb(color_axis), 1)
    ticks = np.arange(-width/2*csize, width/2*csize + 0.5, 1)
    for i in ticks:
        if i == 0:
            continue
        tick_x = int(i / csize)
        cv2.line(img_org, (width//2 - tick_x, height//2-10), (width//2 - tick_x, height//2 + 10), hex_to_rgb(color_axis), 1)
        cv2.line(img_org, (width//2 -10, height//2 - tick_x), (width//2 + 10, height//2- tick_x), hex_to_rgb(color_axis), 1)
    return img_org

def cleanup(fd, urg, md, v=0.0, w=0.0):
    print("Performing cleanup...")
    # 停止速度を送信
    md.send_vw(fd, v, w)
    time.sleep(2)
    # モータードライバをオフにする
    md.turn_off_motors(fd)
    print("Closing serial connection.")
    urg.close()
    #urg_m.close()
    #urg_b.close()

# グローバル変数
# プロセス間共有のため
ox = 0
oy = 0
oa = 0
ou = 0
ov = 0
oth = 0
odo_travel = 0
odo_rotation = 0
urg_data = []
#urg_m_data = []
#urg_b_data = []

#cap = cv2.VideoCapture(0)  # '0' は内蔵カメラ
#frame = ""
#ret = False

stop_thread = False
def blv_odometory_fd(fd):
    global ox, oy, oa, ou, ov, oth, odo_travel, odo_rotation
    prev_ox = 0
    prev_oy = 0
    prev_oa = 0

    file_name = "enclog"
    with open(file_name, "w") as file:
        while not stop_thread:
            ts = int(time.time() * 1e3)
            ox, oy, oa, odo_travel, odo_rotation = md.read_odo2(fd, ox, oy, oa, odo_travel, odo_rotation)
            #print(f"Odo: x={ox:.2f}, y={oy:.2f}, a={oa:.2f}, travel={odo_travel:.2f}, rotation={odo_rotation:.2f}")
            #file.write(f"{ts} {ox} {oy} {oa} end\n")
            dx = ox - prev_ox
            dy = oy - prev_oy
            da = oa - prev_oa
            ou =  dx * cos(oa) + dy * sin(oa)
            ov = -dx * sin(oa) + dy * cos(oa)
            oth = da
            prev_ox, prev_oy, prev_oa = ox, oy, oa
            time.sleep(0.01)

def lidar_measurement_fd(urg, start_angle, end_angle, step_angle, echo_size, fname):
    global urg_data, urg_m_data, urg_b_data
    data_size = 1081*4
    #index = 0 # for LittleSLAM
    with open(fname, "w") as file:
        while not stop_thread:
            ts = int(time.time() * 1e3)
            #file.write(f"LASERSCANRT {ts} {data_size} {start_angle} {end_angle} {step_angle} {echo_size} ")
            if fname == "urglog":
                success, urg_data = urg.one_shot_intensity()
                #file.write(f"LASERSCANRT {ts} {len(urg_data)*4} {start_angle} {end_angle} {step_angle} {echo_size} ")
                #for d in urg_data:
                #    file.write(f"{d[1]} 0 {d[0]} {d[2]} ")
            elif fname == "urglog_m":
                success, urg_m_data = urg.one_shot_intensity()
                #file.write(f"LASERSCANRT {ts} {len(urg_m_data)*4} {start_angle} {end_angle} {step_angle} {echo_size} ")
                #for d in urg_m_data:
                #    file.write(f"{d[1]} 0 0 {d[2]} ")
            elif fname == "urglog_b":
                success, urg_b_data = urg.one_shot_intensity()
                #file.write(f"LASERSCANRT {ts} {len(urg_b_data)*4} {start_angle} {end_angle} {step_angle} {echo_size} ")
                #for d in urg_b_data:
                #    file.write(f"{d[1]} 0 0 {d[2]} ")

            #file.write(f"{ts}\n")

def localization(global_map):
    estimated_pose = [ox, oy, oa] # 並列プロセスで計算しているオドメトリ値
    if len(urg_data) < 1:
        print("Can't get the urg_data. length is zero.")
        return estimated_pose

    estimated_pose = optimize_pose_combined(global_map, mapInfo, urg_data, estimated_pose) # LiDARデータも同様
    return estimated_pose

####### C++ に変更したほうが4倍遅い　
####### Numexpr　でも遅い．おそらく，計算量が有利になるほどのデータ数ではない
import numexpr as ne  # NumPy よりも速い計算ライブラリ
def convert_lidar_to_points(start_angle, end_angle, angle_step, ranges, angles, robot_pose):
    urg_shift_x = 0.165
    x, y, theta = robot_pose

    # ベクトル化とフィルタリング
    valid_mask = (ranges > 300) & (ranges < 50000)
    ranges = ranges[valid_mask]
    angles = angles[valid_mask]

    # 位置計算のベクトル化
    ranges /= 1000
    cos_theta, sin_theta = np.cos(theta), np.sin(theta)
    lx = ranges * np.cos(angles) + urg_shift_x
    ly = ranges * np.sin(angles) 
    gx = x + lx * cos_theta - ly * sin_theta
    gy = y + lx * sin_theta + ly * cos_theta

    return np.column_stack((gx, gy))

# 評価関数をグローバルに移動
searched_pose_list = []
def eval_simple_func(pose, global_map, mapInfo, start_angle, end_angle, angle_step, ranges, angles):
    global searched_pose_list
    sx, sy, sa = pose
    points = convert_lidar_to_points(start_angle, end_angle, angle_step, ranges, angles, (sx, sy, sa))
    eval =  matches_simple(points, global_map, mapInfo)  # 評価値を反転（最小化のため）
    searched_pose_list.append([sx, sy, sa, eval])
    return eval

def matches_simple(points, global_map, mapInfo):
    height, width = global_map.shape[:2]  # 先頭2要素を取得
    #height = len(global_map)
    #width = len(global_map[0])
    # NumPy配列に変換
    points = np.array(points)
    # x, y 座標の変換をベクトル化
    ix = ( points[:, 0] / mapInfo.csize).astype(int) + mapInfo.originX
    iy = (-points[:, 1] / mapInfo.csize).astype(int) + mapInfo.originY
    # 範囲内にあるインデックスをフィルタリング
    valid = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    # 有効なインデックスのみを抽出
    ix_valid = ix[valid]
    iy_valid = iy[valid]
    # マップの値が 0 の場合のみカウント
    eval = np.sum(global_map[iy_valid, ix_valid] == 0)

    return -eval

def optimize_pose_combined(global_map, mapInfo, urg_data, robot_pose):
    global searched_pose_list
    searched_pose_list = []
    N = 1 # Nを10倍すると推定時間は1/2になった．劇的な効果ではない
    index, ranges, _ = np.array(urg_data).T[:, ::N]
    #index, ranges, _ = np.array(urg_data).T
    ranges = ranges.astype(float)
    angles = np.radians([ind * step_angle + start_angle for ind in index])

    # 粗い探索
    # 今の動作周期だと，この範囲では探索から外れてしまう．探索時間を減らす工夫のほうが必要
    bounds = [(robot_pose[0] - 0.5, robot_pose[0] + 0.5),
              (robot_pose[1] - 0.5, robot_pose[1] + 0.5),
              (robot_pose[2] - 20*math.pi/180, robot_pose[2] + 20*math.pi/180)]
    result_de = differential_evolution(
        eval_simple_func,
        bounds,
        args=(global_map, mapInfo, start_angle, end_angle, step_angle, ranges, angles),
        strategy='best1exp',
        popsize=10,     # 自己位置推定の高速化のために下げた．ベイズ最適化で求めた86よりちょっと高くする
        tol=1e-4,
        maxiter=18,     # こちらも下げた．同じく，38よりちょっと高くする
        workers=1,
        updating='immediate'
    )

    return result_de.x

def draw_lidar_on_global_map(img_disp, urg_data, rx, ry, ra, mapInfo):
    color = hex_to_rgb(config.map.color.target)
    height = len(global_map)
    width = len(global_map[0])
    # LiDAR変換用にcos, sin のリストを作る
    cs = [cos((i * step_angle + start_angle)*pi/180 + ra) for i in range(int((end_angle - start_angle)/step_angle) + 1)]
    sn = [sin((i * step_angle + start_angle)*pi/180 + ra) for i in range(int((end_angle - start_angle)/step_angle) + 1)]
    d_values = np.array([d[1] for d in urg_data])  # LiDARデータの距離成分を抽出
    try:
        xd = d_values * cs / 1000 + rx # X座標変換
        yd = d_values * sn / 1000 + ry # Y座標変換
        ix = ( xd / mapInfo.csize + mapInfo.originX).astype(int)
        iy = (-yd / mapInfo.csize + mapInfo.originY).astype(int) 
        # 範囲内の座標をフィルタリング
        valid_mask = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
        ix = ix[valid_mask]
        iy = iy[valid_mask]
        # 描画
        for x, y in zip(ix, iy):
            cv2.circle(img_disp, (x, y), 5, color, -1)
    except:
        print("d_valueにエラーの可能性")
        print(d_values)
    return img_disp

cap = cv2.VideoCapture(0)  # '0' は内蔵カメラ
frame = None
ret = False
def image_writer():
    pass
    #global ret, frame
    #dir_name = datetime.now().strftime("%y%m%d")
    #dir_name += "_images"
    #os.makedirs(dir_name, exist_ok=True)
    #while not stop_thread:
    #    ts = int(time.time() * 1e3)
    #    ret, frame = cap.read()
    #    cv2.imwrite(f"./{dir_name}_images/{ts}.png", frame)
    #    time.sleep(1.0)

def get_wp_list():
    wp_list = []
    wp_list.append([5.0, -0.5, 0])
    wp_list.append([5.0, -2.0, -np.pi/2])
    wp_list.append([0.0, -2.0, -np.pi])
    #wp_list.append([0.0, 0.0, np.pi/2])
    return wp_list
    #with open(wp_fname, "r") as file:

def get_navigation_cmd(estimated_pose, wp_list, wp_index):
    x, y, a = estimated_pose
    wx, wy, wa = wp_list[wp_index]
    d = math.sqrt((wx - x)**2 + (wy - y)**2)
    da = wa - a
    if da > np.pi:
        da -= 2*np.pi
    elif da < -np.pi:
        da += 2*np.pi
    #if d < 1.0 and abs(da) < 15*np.pi/180:
    if d < 1.0:
        wp_index += 1
        wx, wy, wa = wp_list[wp_index]
        md.send_vw(fd, 0, 0)
        time.sleep(2.0)
    target_r = d
    target_a = np.atan2(wy - y, wx - x)
    if target_a > np.pi:
        target_a -= 2*np.pi
    elif target_a < -np.pi:
        target_a += 2*np.pi
    robot_a = wa

    if target_r > 0.5:
        v = 0.4
    else:
        v = 0
    if target_a > 5*np.pi/180:
        w = target_a
    elif target_a < -5*np.pi/180:
        w = target_a
    else:
        w = 0
    w = 0.2*w
    print(f"{v}, {w:.3f}, {x:.3f}, {y:.3f}, {wx}, {wy}, {target_r:.3f}, {target_a:.3f}")
    return v, w, target_r, target_a, robot_a, wp_index
try:
    # Initialize Oriental motor BLV-R 
    fd = md.init(config.motor.serial_port, config.motor.baudrate)

    # fd に対してアクセスをする別の並列スレッドを立ち上げる
    blv_odometory_thread = threading.Thread(target=blv_odometory_fd, args=(fd,))
    blv_odometory_thread.start()

    md.turn_on_motors(fd)

    start_angle = config.lidar.start_angle
    end_angle   = config.lidar.end_angle
    step_angle  = config.lidar.step_angle
    echo_size   = config.lidar.echo_size
    lidar_measurement_thread = threading.Thread(target=lidar_measurement_fd, args=(urg, start_angle, end_angle, step_angle, echo_size, "urglog",))
    lidar_measurement_thread.start()
    #lidar_m_measurement_thread = threading.Thread(target=lidar_measurement_fd, args=(urg_m, start_angle, end_angle, step_angle, echo_size, "urglog_m",))
    #lidar_m_measurement_thread.start()
    #lidar_b_measurement_thread = threading.Thread(target=lidar_measurement_fd, args=(urg_b, start_angle, end_angle, step_angle, echo_size, "urglog_b",))
    #lidar_b_measurement_thread.start()

    # カメラ画像取得のプロセス起動
    camera_thread = threading.Thread(target=image_writer, args=())
    camera_thread.start()

    height = config.map.window_height
    width = config.map.window_width
    csize = config.map.csize
    img_org = make_img_org(height, width, config.map.color.bg, config.map.color.axis, csize)
    img = copy.deepcopy(img_org)
    map = copy.deepcopy(img_org)
    cv2.imshow("LiDAR", img)
    cv2.imshow("Map", map)
    global_map = cv2.imread(f"{DIR_NAME}/opt.png")
    cv2.imshow("GlobalMap", global_map)

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
            print("LiDARデータは保存されません．urglog は保存されます")
            res = "y"
            #res = input("LiDARデータは保存されません．本当に良いですか？ [y/N]")
            if res == "" or res == "n" or res == "N":
                v = 0.0
                w = 0.0
                md.send_vw(fd, v, w)
                time.sleep(2)
                
                md.turn_off_motors(fd)
                
                urg.close()
                #urg_m.close()
                #urg_b.close()
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
    NUM_JOY_MAPPING    = 12

    H_normalized = np.loadtxt("./Hmatrix.txt")
    H_inv = np.linalg.inv(H_normalized)
    H_inv = H_inv/H_inv[2][2]
    ## 中心(1500, 0)の半径500の円上の点のリスト
    center_x = 0
    center_y = 0
    point_on_circle = [(center_x + 50 * cos(i * pi / 180), center_y + 50 * sin(i * pi / 180)) for i in range(360)]
    #cap = cv2.VideoCapture(0)  # '0' は内蔵カメラ
    # LiDAR変換用にcos, sin のリストを作る
    cs = [cos((i * step_angle + start_angle)*pi/180) for i in range(int((end_angle - start_angle)/step_angle) + 1)]
    sn = [sin((i * step_angle + start_angle)*pi/180) for i in range(int((end_angle - start_angle)/step_angle) + 1)]
    # 色をNumPyで表現
    color = np.array(hex_to_rgb(config.map.color.point), dtype=np.uint8)
    # 現在地図上での自己位置
    rx = 0
    ry = 0
    ra = 0
    # 最初の地図は強制的に登録する
    #success, urg_data = urg.one_shot_intensity()
    #if success:
    #map, _ = draw_lidar_on_img(img_org, urg_data, cs, sn)

    ########################################
    # Main loop start
    ########################################
    loop_counter = 0
    wp_index = 0
    wp_list = get_wp_list()
    prev_odo_x = ox 
    prev_odo_y = oy
    prev_odo_a = oa
    while True:
        ts = int(time.time() * 1e3)
        
        # Localization
        # Global map を用いて自己位置推定
        s = time.time()
        cur_x = 
        cur_x, cur_y, cur_a = ox, oy, oa

        estimated_pose = localization(global_map) 
        rx, ry, ra = estimated_pose # 現在姿勢
        DT = time.time() - s
        print(f"Localization time: {DT}")
        search_result_path = f"search_result/search_result_{loop_counter}.txt"
        with open(search_result_path, "w") as file:
            for val in searched_pose_list:
                file.write(f"{val[0]} {val[1]} {val[2]} {val[3]}\n")
                # val[0-2] xya
                # val[3] eval
        searched_pose_list_np = np.array(searched_pose_list)
        de_cov_matrix = np.cov(searched_pose_list_np[:, :3], rowvar=False)
        dx = cur_x - prev_odo_x
        dy = cur_y - prev_odo_y
        da = cur_a - prev_odo_a 
        dist = math.sqrt(dx**2 + dy**2)
        vt = dist / DT
        wt = abs(da / DT)
        if vt < 0.02:
            vt = 0.02
        if wt < 0.05:
            wt = 0.05
        odo_cov_matrix = np.eye(3)
        odo_cov_matrix[0, 0] = 0.01 * vt*vt
        odo_cov_matrix[1, 1] = 0.05 * vt*vt
        odo_cov_matrix[2, 2] = 0.5 * wt*wt
        # 状態ベクトル
        x_odo = np.array([cur_x, cur_y, cur_a])
        x_scan = np.array([rx, ry, ra])
        # カルマンゲインの計算
        K = odo_cov_matrix @ np.linalg.inv(odo_cov_matrix + de_cov_matrix)
        # センサフュージョン
        x_fused = x_odo + K @ (x_scan - x_odo)
        # 共分散行列の更新
        I = np.eye(3)
        P_fused = (I - K) @ odo_cov_matrix

        print("融合後の位置:", x_fused)
        print("融合後の共分散行列:\n", P_fused)
        
        prev_odo_x = cur_x 
        prev_odo_y = cur_y
        prev_odo_a = cur_a

        rx = x_fused[0]
        ry = x_fused[1]
        ra = x_fused[2]
        loop_counter += 1

        # python と cpp で描画更新速度の比較
        #s = time.time()
        #img_disp = copy.deepcopy(global_map)
        #ix = int( rx / mapInfo.csize) + mapInfo.originX
        #iy = int(-ry / mapInfo.csize) + mapInfo.originY
        #cv2.circle(img_disp, (ix, iy), 25, (82, 54, 20), -1)
        #draw_lidar_on_global_map(img_disp, urg_data, rx, ry, ra, mapInfo)
        #print(f"Drawing Global map time (Python): {time.time()-s}")
        #s = time.time()
        img_disp = copy.deepcopy(global_map)
        # 描画をcppで実行しても，速度差はない
        map_info = lidar_draw.MapInfo()
        map_info.originX = mapInfo.originX
        map_info.originY = mapInfo.originY
        map_info.csize = mapInfo.csize
        color_hex = "#FFFFFF"  # White
        lidar_draw.draw_lidar_on_global_map(img_disp, urg_data, rx, ry, ra, map_info, start_angle, end_angle, step_angle, color_hex)
        #print(f"Drawing Global map time (cpp): {time.time()-s}")
        cv2.imshow("GlobalMap", img_disp)
            
        # Get & Show LiDAR data
        #success, urg_data = urg.one_shot()
        #if success:
        #img, d = draw_lidar_on_img(img_org, urg_data, cs, sn)

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
                    break

            # Get button state of joystick
            button_pressed_turn_left  = joystick.get_button(NUM_JOY_TURN_LEFT)  
            button_pressed_go_forward = joystick.get_button(NUM_JOY_GO_FORWARD)  
            button_pressed_go_back    = joystick.get_button(NUM_JOY_GO_BACK) 
            button_pressed_turn_right = joystick.get_button(NUM_JOY_TURN_RIGHT) 
            button_pressed_shutdown   = joystick.get_button(NUM_JOY_SHUTDOWN)
            button_pressed_mapping    = joystick.get_button(NUM_JOY_MAPPING)

        if button_pressed_turn_left: # Turn Left
            target_r  = 1.0
            target_a  = pi/2
            robot_a   = pi/2
            center_y += 100
            #print("0ボタンが押されています")
        elif button_pressed_go_forward: # Go forward
            target_r  = 1.0
            target_a  = 0.0
            robot_a   = 0.0
            center_x += 100
            #print("1ボタンが押されています")
        elif button_pressed_go_back: # Go back
            target_r  = 1.0
            target_a  = pi
            robot_a   = 0.0
            center_x -= 100
            #print("2ボタンが押されています")
        elif button_pressed_turn_right: # Turn Right
            target_r  = 1.0
            target_a  = -pi/2
            robot_a   = -pi/2
            center_y -= 100
            #print("3ボタンが押されています")
        elif button_pressed_shutdown: # Shutdown
            print("Pressed Stop button")
            stop_thread = True
            blv_odometory_thread.join()
            lidar_measurement_thread.join()
            camera_thread.join()
            #lidar_m_measurement_thread.join()
            #lidar_b_measurement_thread.join()
            break
        elif button_pressed_mapping:
            print("Mapを更新します")
            map, _ = draw_lidar_on_img(img_org, urg_data, cs, sn)
        else:
            # joypadの入力がない場合は，現在位置を目標値点としたmpcを行う
            target_r = 0.0
            target_a = 0.0
            robot_a = 0.0

        if NAVI:
            v, w, target_r, target_a, robot_a, wp_index = get_navigation_cmd(estimated_pose, wp_list, wp_index)
            md.send_vw(fd, v, w)

        # Get control command from MPC
        x_ref = casadi.DM([target_r*cos(target_a), target_r*sin(target_a),  robot_a])
        u_ref = casadi.DM([0, 0])
        K = 10
        T = 1
        x_init = casadi.DM([0, 0, 0])   # 常に現在のロボット座標系からスタートする
        mpc = MPC.MPCController(x_ref, u_ref, K, T)
        x0 = casadi.DM.zeros(mpc.total_vars)
        x_current = x_init
        u_opt, x0 = mpc.compute_optimal_control(x_current, x0)
        md.send_vw(fd, u_opt[0], u_opt[1])

        ## MPCに与えたターゲット座標
        #tx =  int(-x_ref[1, 0]/csize) + width//2
        #ty = -int( x_ref[0, 0]/csize) + height//2
        #cx =  int(-center_y/1000/csize) + width//2
        #cy = -int( center_x/1000/csize) + height//2
        #cv2.circle(img, (tx, ty), int(1/config.map.csize/2), hex_to_rgb(config.map.color.target), 3)
        ##cv2.circle(img, (cx, cy), 50, (0, 0, 255), 1)
        #cv2.imshow("LiDAR", img)

        #lx =  int(-oy/csize) + width//2
        #ly = -int( ox/csize) + height//2
        #cv2.circle(map, (lx, ly), 10, (255, 0, 0), 2)
        #cv2.imshow("Map", map)

        # カメラ画像内にターゲット情報を表示する
        # Hmatrixが，mm座標系とpixel座標系の相互変換で定義されていることに注意する
        # mm座標系におけるtarget_x, y を中心とする円を作成し，H_inv を用いてカメラ画像上へ射影変換する
        #ret, frame = cap.read()
                
        #target_x = target_r*cos(target_a) * 1000    # m to mm
        #target_y = target_r*sin(target_a) * 1000    # m to mm
        #point_on_circle = [(target_x + 50 * cos(i * pi / 180), target_y + 50 * sin(i * pi / 180)) for i in range(360)]
        #for tx, ty in point_on_circle:
        #    p1 = np.array([tx, ty, 1])
        #    p_origin = np.dot(H_inv, p1)
        #    p_origin = p_origin/p_origin[2]
        #    cv2.circle(frame, (int(p_origin[0]), int(p_origin[1])), 2, (0, 0, 255), -1)
        #cv2.imshow('Capture image', frame)

        cv2.waitKey(5)
        pygame.time.wait(10)

except KeyboardInterrupt:
    print("Pressed Ctrl + C")
    print("Shutting down now...")

except Exception as e:
    print(f"An error occurred: {e}")
    traceback.print_exc()  # エラーの詳細情報（トレースバック）を表示

finally:
    print("Cleanup")
    stop_thread = True
    blv_odometory_thread.join()
    lidar_measurement_thread.join()
    camera_thread.join()
    #lidar_m_measurement_thread.join()
    #lidar_b_measurement_thread.join()
    cleanup(fd, urg, md)
    cv2.destroyAllWindows()
    cap.release()
    pygame.quit()
    sys.exit()