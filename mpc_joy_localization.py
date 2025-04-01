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
from scipy.optimize import differential_evolution
from pathlib import Path # pathの操作
import readchar # 1文字の入力受付
import shutil   # sub-dir の操作

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

# Select navigation mode
NAVI = True # enable navigation
#NAVI = False # disable navigation

STORE_ROOT_DIR_NAME = "experiment_250328-2"
if Path(STORE_ROOT_DIR_NAME).exists():
    print(f"'{STORE_ROOT_DIR_NAME}' exists.")
    print("Do you want to continue? y/[n] ", end="", flush=True)
    response = readchar.readchar()  # 1文字入力を受け付ける
    if response.lower() == "y":
        print("\nContinuing...")
        print("Delete existing log files")
        files_to_delete = ["enclog", "estimated_pose.txt", "mpc_joy_localization_output.mp4", "urglog"]
        for file in files_to_delete:
            file_path = os.path.join(STORE_ROOT_DIR_NAME, file)
            if os.path.exists(file_path):
                os.remove(file_path)
                print(f"Deleted: {file_path}")
        subdir = os.path.join(STORE_ROOT_DIR_NAME, "search_result")  # サブディレクトリのパス
        if os.path.exists(subdir):
            for file in os.listdir(subdir):
                file_path = os.path.join(subdir, file)
                if os.path.isfile(file_path):  # ファイルなら削除
                    os.remove(file_path)
                    print(f"Deleted: {file_path}")
            print("✅ Delete existing log files")
        # 必要なファイルが存在するか確認
        else:
            # search_resultが無ければ作成
            os.makedirs(subdir, exist_ok=True)
            print(f"✅ make {subdir}")

        if os.path.isdir(os.path.join(STORE_ROOT_DIR_NAME, "global_map")):
            print("✅ exist global_map")
        else:
            print("global_mapを作成してください．")
            exit()
        if os.path.isfile(os.path.join(STORE_ROOT_DIR_NAME, "wp_list.txt")):
            print("✅ exist wp_list.txt")
        else:
            print("wp_list.txtを作成してください．")
            exit()
    else:
        print("\nExiting...")
        exit()
else:
    print(f"'{STORE_ROOT_DIR_NAME}' does not exist.")
    # ディレクトリがなければ作成
    os.makedirs(STORE_ROOT_DIR_NAME, exist_ok=True)
    SUBDIRS = ["search_result"]  # 作成するサブディレクトリのリスト
    # サブディレクトリの作成
    for subdir in SUBDIRS:
        os.makedirs(os.path.join(STORE_ROOT_DIR_NAME, subdir), exist_ok=True)
    print(f"{STORE_ROOT_DIR_NAME}は作成しました．")
    print("global_map, wp_list.txtを作成してから実行してください")
    exit()

# 実験ノートを作成
EXPERIMENT_NOTE_PATH = f"{STORE_ROOT_DIR_NAME}/note.txt"
experiment_note_file = open(EXPERIMENT_NOTE_PATH, "w")
ts = time.time()  # 現在のタイムスタンプ
dt = datetime.fromtimestamp(ts)  # datetimeオブジェクトに変換
TODAY_TIME = dt.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # ミリ秒を3桁に調整
experiment_note_file.write(f"DATE: {TODAY_TIME}\n")
experiment_note_file.write(f"Map: {STORE_ROOT_DIR_NAME}/global_map/rebuild_opt.png\n")
experiment_note_file.write(f"Map Info: {STORE_ROOT_DIR_NAME}/global_map/mapInfo.lua\n")

# Include default configuration
config = rc.read_config('config.lua')
mapInfo = rc.read_mapInfo(f"{STORE_ROOT_DIR_NAME}/global_map/mapInfo.lua")

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

stop_thread = False
def blv_odometory_fd(fd):
    global ox, oy, oa, ou, ov, oth, odo_travel, odo_rotation
    prev_ox = 0
    prev_oy = 0
    prev_oa = 0

    file_name = f"{STORE_ROOT_DIR_NAME}/enclog"
    with open(file_name, "w") as file:
        while not stop_thread:
            ts = int(time.time() * 1e3)
            ox, oy, oa, odo_travel, odo_rotation = md.read_odo2(fd, ox, oy, oa, odo_travel, odo_rotation)
            #print(f"Odo: x={ox:.2f}, y={oy:.2f}, a={oa:.2f}, travel={odo_travel:.2f}, rotation={odo_rotation:.2f}")
            file.write(f"{ts} {ox} {oy} {oa} end\n")
            dx = ox - prev_ox
            dy = oy - prev_oy
            da = oa - prev_oa
            ou =  dx * cos(oa) + dy * sin(oa)
            ov = -dx * sin(oa) + dy * cos(oa)
            oth = da
            prev_ox, prev_oy, prev_oa = ox, oy, oa
            time.sleep(0.05)

def lidar_measurement_fd(urg, start_angle, end_angle, step_angle, echo_size, fname):
    global urg_data, urg_m_data, urg_b_data
    global ox, oy, oa, ou, ov, oth, odo_travel, odo_rotation
    data_size = 1081*4
    #index = 0 # for LittleSLAM
    filename = f"{STORE_ROOT_DIR_NAME}/{fname}"
    with open(filename, "w") as file:
        while not stop_thread:
            ts = int(time.time() * 1e3)
            if fname == "urglog":
                success, urg_data = urg.one_shot_intensity()
                if success:
                    file.write(f"LASERSCANRT {ts} {len(urg_data)*4} {start_angle} {end_angle} {step_angle} {echo_size} ")
                    for d in urg_data:
                        file.write(f"{d[1]} 0 {d[0]} {d[2]} ")
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

            file.write(f"{ox:.3f} {oy:.3f} {oa:.3f} {ts}\n")

def localization(global_map, cur_x, cur_y, cur_a):
    estimated_pose = [cur_x, cur_y, cur_a] 
    if len(urg_data) < 1:
        print("Can't get the urg_data. length is zero.")
        return estimated_pose

    estimated_pose = optimize_pose_combined(global_map, mapInfo, urg_data, estimated_pose) # LiDARデータも同様

    # 以下のcppライブラリは動作速度が遅すぎて使えない
    # 原因調査は保留
    #N = 10
    #index, ranges, _ = np.array(urg_data).T[:, ::N]
    #ranges = ranges.astype(float)
    #ranges /= 1000.0
    #angles = np.radians([ind * 0.25 - 135.0 for ind in index])
    #estimated_pose = my_robot_localization.optimize_pose_de(global_map,
    #                               mapInfo.csize, mapInfo.originX, mapInfo.originY, -135.0, 135.0, 0.25, 
    #                               ranges, angles, estimated_pose, 10, 18, 0.8, 0.9) 
    succ = True
    threshold = 20
    if searched_pose_list[-1][3] > (-threshold):
        print(f"一致点が少ない {searched_pose_list[-1][3]}")
        estimated_pose = [cur_x, cur_y, cur_a] 
        succ = False
    min_threshold = 50   # 以上の値
    max_threshold = 35000  # 未満の値
    count = sum(1 for index, r, intensity in urg_data if min_threshold <= r < max_threshold)
    return estimated_pose, succ, count

####### C++ に変更したほうが4倍遅い　
####### Numexpr　でも遅い．おそらく，計算量が有利になるほどのデータ数ではない
# import numexpr as ne  # NumPy よりも速い計算ライブラリ
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
    N = min(250, len(points)) # 抽出する点の数
    indices = np.random.choice(points.shape[0], N, replace=False)  # 重複なし
    points = points[indices]
    eval =  matches_simple(points, global_map, mapInfo)  # 評価値を反転（最小化のため）
    searched_pose_list.append([sx, sy, sa, eval])
    return eval

def matches_simple(points, global_map, mapInfo):
    height, width = global_map.shape[:2]  # 先頭2要素を取得
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
    N = 10 # Nを10倍すると推定時間は1/2になった．劇的な効果ではない
    index, ranges, _ = np.array(urg_data).T[:, ::N]
    #index, ranges, _ = np.array(urg_data).T
    ranges = ranges.astype(float)
    angles = np.radians([ind * step_angle + start_angle for ind in index])

    dxy = 0.5
    da = 10*math.pi/180
    # 粗い探索
    # 今の動作周期だと，この範囲では探索から外れてしまうことがある．探索時間を減らす工夫のほうが必要
    bounds = [(robot_pose[0] - dxy, robot_pose[0] + dxy),
              (robot_pose[1] - dxy, robot_pose[1] + dxy),
              (robot_pose[2] - da, robot_pose[2] + da)]
    result_de = differential_evolution(
        eval_simple_func,
        bounds,
        args=(global_map, mapInfo, start_angle, end_angle, step_angle, ranges, angles),
        strategy='best1exp',
        popsize=20,     # 自己位置推定の高速化のために下げた．ベイズ最適化で求めた86よりちょっと高くする
        tol=1e-4,
        maxiter=15,     # こちらも下げた．同じく，38よりちょっと高くする
        workers=1,
        updating='immediate'
    )

    return result_de.x

# 以下はcppライブラリに移植したので使っていない
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

cap = cv2.VideoCapture(1)  # '0' は内蔵カメラ
frame = None
ret = False
def image_writer():
    global ret, frame
    #dir_name = datetime.now().strftime("%y%m%d")
    #dir_name += "_images"
    #os.makedirs(dir_name, exist_ok=True)
    while not stop_thread:
        ts = int(time.time() * 1e3)
        ret, frame = cap.read()
        #cv2.imwrite(f"./{dir_name}_images/{ts}.png", frame)
        time.sleep(1.0)

def get_wp_list():
    wp_filepath = STORE_ROOT_DIR_NAME + "/wp_list.txt"
    wp_list = []
    with open(wp_filepath, 'r') as file:
        for line in file:
            parts = line.strip().split(',')
            x = float(parts[0])
            y = float(parts[1])
            wp_list.append([x, y])

    return wp_list

def get_navigation_cmd(estimated_pose, wp_list, wp_index, global_map):
    x, y, a = estimated_pose
    wx, wy = wp_list[wp_index]
    target_r = math.sqrt((wx - x)**2 + (wy - y)**2)
    delta_th = math.atan2(wy-y, wx-x) - a # 現在の方向を基準にした，wpまでの方向差
    if delta_th > np.pi:
        delta_th -= 2*np.pi
    elif delta_th < -np.pi:
        delta_th += 2*np.pi
    # WPの到達判定
    if target_r < 1.0:
        wp_index += 1
        wx, wy = wp_list[wp_index]
        md.send_vw(fd, 0, 0)
        time.sleep(2.0)
        # WP到達毎に自己推定の再確認をする
        checked_pose, success_flag, valid_count = localization(global_map, x, y, a) 
        if success_flag:
            estimated_pose = checked_pose

    if target_r > 0.5:
        v = 0.5
        #v = 0.45
    else:
        v = 0
    w = 0.5*delta_th
    print(f"v:{v:.3f}, w[deg/s]:{w*180/np.pi:.1f}, x:{x:.3f}, y:{y:.3f}, a[deg]:{a*180/np.pi:.3f}, wx:{wx:.3f}, wx:{wy:.3f}, dist:{target_r:.3f}, th[deg]:{delta_th*180/np.pi:.1f}")
    return v, w, target_r, delta_th, wp_index, estimated_pose

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
    global_map = cv2.imread(f"{STORE_ROOT_DIR_NAME}/global_map/rebuild_opt.png")
    cv2.imshow("GlobalMap", global_map)

    # MP4動画に保存する処理
    # 動画ライターの設定
    output_file = STORE_ROOT_DIR_NAME + "/mpc_joy_localization_output.mp4"
    height_img_disp_color, width_img_disp_color = global_map.shape[:2]
    height, width = frame.shape[:2]  # 先頭2要素を取得
    fps = 30  # フレームレート
    # MP4動画に保存準備
    fourcc = cv2.VideoWriter_fourcc(*'H264')  # コーデック指定
    out = cv2.VideoWriter(output_file, fourcc, fps, (width_img_disp_color+width, height_img_disp_color))

    # Create file name to store LiDAR data
    if config.lidar.store_data:
        #file_name = input("データを保存するファイル名を指示してください（終了する場合はEnterキーを押してください）: ")
        timestamp = int(time.time())
        formatted_date = datetime.fromtimestamp(timestamp).strftime('%Y_%m_%d_%H_%M_%S')
        file_name = STORE_ROOT_DIR_NAME + "/urglog_" + formatted_date

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

    ########################################
    # Main loop start
    ########################################
    loop_counter = 0
    wp_index = 0
    wp_list = get_wp_list()
    ave_valid_count = 0
    with open(f"{STORE_ROOT_DIR_NAME}/estimated_pose.txt", "w") as est_file:
        odo_prev_x, odo_prev_y, odo_prev_a = ox, oy, oa
        while True:
            ts = int(time.time() * 1e3)

            # Localization
            # Global map を用いて自己位置推定
            s = time.time()
            # オドメトリで現在位置を更新
            odo_delta_x = ox - odo_prev_x
            odo_delta_y = oy - odo_prev_y
            odo_delta_a = oa - odo_prev_a
            odo_u =  odo_delta_x * cos(oa) + odo_delta_y * sin(oa)
            odo_v = -odo_delta_x * sin(oa) + odo_delta_y * cos(oa)
            odo_th = odo_delta_a
            odo_prev_x, odo_prev_y, odo_prev_a = ox, oy, oa

            cur_x = math.cos(ra) * odo_u - math.sin(ra) * odo_v + rx
            cur_y = math.sin(ra) * odo_u + math.cos(ra) * odo_v + ry
            cur_a = odo_th + ra
            print(f"オドメトリ更新値 {cur_x:.3f} {cur_y:.3f} {cur_a*180/np.pi:.1f} {odo_u:.3f} {odo_v:.3f} {odo_th*180/np.pi:.1f}")

            estimated_pose, success_flag, valid_count = localization(global_map, cur_x, cur_y, cur_a) 
            ave_valid_count = 0.5*(ave_valid_count + valid_count)
            # 強制的にオドメトリ値にする場合
            # estimated_pose = [cur_x, cur_y, cur_a]

            rx, ry, ra = estimated_pose # 現在姿勢
            DT = time.time() - s
            print(f"Localization time: {DT:.3f}")
            search_result_path = f"{STORE_ROOT_DIR_NAME}/search_result/search_result_{loop_counter}.txt"
            with open(search_result_path, "w") as file:
                for val in searched_pose_list:
                    file.write(f"{val[0]} {val[1]} {val[2]} {val[3]}\n")
                    # val[0-2] xya
                    # val[3] eval
            est_file.write(f"{rx} {ry} {ra} {searched_pose_list[-1][3]} {ts} {success_flag} {ave_valid_count}\n") 
            if 0:
                searched_pose_list_np = np.array(searched_pose_list)
                de_cov_matrix = np.cov(searched_pose_list_np[:, :3], rowvar=False)
                dist = ou
                vt = dist / DT
                wt = abs(oth / DT)
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
            if len(urg_data) > 0:
                # 描画をcppで実行しても，速度差はない
                map_info = lidar_draw.MapInfo()
                map_info.originX = mapInfo.originX
                map_info.originY = mapInfo.originY
                map_info.csize = mapInfo.csize
                color_hex = "#FFFFFF"  # White
                lidar_draw.draw_lidar_on_global_map(img_disp, urg_data, rx, ry, ra, map_info, start_angle, end_angle, step_angle, color_hex)

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
                md.send_vw(fd, 0, 0)
                time.sleep(1)
                stop_thread = True
                blv_odometory_thread.join()
                lidar_measurement_thread.join()
                camera_thread.join()
                #lidar_m_measurement_thread.join()
                #lidar_b_measurement_thread.join()
                break
            elif button_pressed_mapping:
                print("Pause...")
                md.send_vw(fd, 0, 0)
                time.sleep(5)
                while True:
                    pygame.time.wait(10)
                    BRAKE_FLAG = False
                    for event in pygame.event.get():
                        if event.type == pygame.JOYBUTTONDOWN:
                            if event.button == NUM_JOY_MAPPING:
                                BRAKE_FLAG = True
                                break
                    if BRAKE_FLAG:
                        break
            else:
                # joypadの入力がない場合は，現在位置を目標値点としたmpcを行う
                target_r = 0.0
                target_a = 0.0
                robot_a = 0.0

            #img_disp_color = cv2.cvtColor(img_disp, cv2.COLOR_GRAY2BGR)
            img_disp_color = img_disp
            if NAVI:
                v, w, target_r, target_a, wp_index, estimated_pose = get_navigation_cmd(estimated_pose, wp_list, wp_index, global_map)
                # Get control command from MPC
                # 以下はうまくいっていない
                if 0:
                    x_ref = casadi.DM([target_r*cos(target_a), target_r*sin(target_a),  robot_a])
                    u_ref = casadi.DM([0, 0])
                    K = 10
                    T = 2
                    x_init = casadi.DM([0, 0, 0])   # 常に現在のロボット座標系からスタートする
                    mpc = MPC.MPCController(x_ref, u_ref, K, T)
                    x0 = casadi.DM.zeros(mpc.total_vars)
                    x_current = x_init
                    u_opt, x0 = mpc.compute_optimal_control(x_current, x0)
                    v, w = u_opt
                    #md.send_vw(fd, u_opt[0], u_opt[1])
                md.send_vw(fd, v, w)
                # wp_listを表示
                for wp in wp_list:
                    x, y = wp
                    ix = int( x/mapInfo.csize + mapInfo.originX)
                    iy = int(-y/mapInfo.csize + mapInfo.originY)
                    cv2.circle(img_disp_color, (ix, iy), 10, (255, 0, 0), 2, cv2.LINE_AA)
                # ターゲットのwp
                tx, ty = wp_list[wp_index]
                ix = int( tx/mapInfo.csize + mapInfo.originX)
                iy = int(-ty/mapInfo.csize + mapInfo.originY)
                cv2.circle(img_disp_color, (ix, iy), 20, (255, 0, 0), -1, cv2.LINE_AA)
                # 推定座標
                ix = int( rx/mapInfo.csize + mapInfo.originX)
                iy = int(-ry/mapInfo.csize + mapInfo.originY)
                cv2.circle(img_disp_color, (ix, iy), 40, (0, 255, 0), 5, cv2.LINE_AA)

            else:
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
            
            # カメラ画像と地図画像を統合する
            img_integrated = np.zeros((height_img_disp_color, width_img_disp_color + width, 3), dtype=np.uint8)
            img_integrated[0:height_img_disp_color, 0:width_img_disp_color] = img_disp_color
            img_integrated[0:height, width_img_disp_color:width_img_disp_color+width] = frame
            # 日付
            ts = time.time()  # 現在のタイムスタンプ
            dt = datetime.fromtimestamp(ts)  # datetimeオブジェクトに変換
            text_date = dt.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # ミリ秒を3桁に調整
            # タイムスタンプの取得
            text_ts = str(f"ts:{int((ts*1e3))}")
            text_pose = str(f"x:{rx:.3f} y:{ry:.3f} a:{ra*180/np.pi:.1f}[deg]")
            text_travel = str(f"travel:{odo_travel:.1f}[m]")
            text_vw = str(f"v:{v:.2f}[m/s] w:{w*180/np.pi:.1f}[deg/s]")
            # フォント設定
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 5
            font_thickness = 2
            text_color = (255, 255, 255)  # 白

            # テキストのサイズを取得
            (text_width, text_height), _ = cv2.getTextSize(text_ts, font, 2, font_thickness)
            # 右下の座標を決定（余白 10 ピクセルを確保）
            x = img_integrated.shape[1] - text_width - 10
            y = img_integrated.shape[0] - 20  # 下端から 50 ピクセル上
            # テキストを描画
            cv2.putText(img_integrated, text_ts, (x, y), font, 2, text_color, font_thickness)

            # テキストのサイズを取得
            (text_width, text_height), _ = cv2.getTextSize(text_date, font, font_scale, font_thickness)
            # 右下の座標を決定（余白 10 ピクセルを確保）
            x = img_integrated.shape[1] - text_width - 10
            y = img_integrated.shape[0] - 210  # 下端から 10 ピクセル上
            # テキストを描画
            cv2.putText(img_integrated, text_date, (x, y), font, font_scale, text_color, font_thickness)

            # テキストのサイズを取得
            (text_width, text_height), _ = cv2.getTextSize(text_pose, font, font_scale, font_thickness)
            # 右下の座標を決定（余白 10 ピクセルを確保）
            x = img_integrated.shape[1] - text_width - 10
            y = img_integrated.shape[0] - 410  # 下端から 50 ピクセル上
            # テキストを描画
            cv2.putText(img_integrated, text_pose, (x, y), font, font_scale, text_color, font_thickness)

            # テキストのサイズを取得
            (text_width, text_height), _ = cv2.getTextSize(text_travel, font, font_scale, font_thickness)
            # 右下の座標を決定（余白 10 ピクセルを確保）
            x = img_integrated.shape[1] - text_width - 10
            y = img_integrated.shape[0] - 610  # 下端から 50 ピクセル上
            # テキストを描画
            cv2.putText(img_integrated, text_travel, (x, y), font, font_scale, text_color, font_thickness)

            # テキストのサイズを取得
            (text_width, text_height), _ = cv2.getTextSize(text_vw, font, font_scale, font_thickness)
            # 右下の座標を決定（余白 10 ピクセルを確保）
            x = img_integrated.shape[1] - text_width - 10
            y = img_integrated.shape[0] - 810  # 下端から 50 ピクセル上
            # テキストを描画
            cv2.putText(img_integrated, text_vw, (x, y), font, font_scale, text_color, font_thickness)

            out.write(img_integrated)
            cv2.imshow("GlobalMap", img_integrated)

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
    out.release()
    print(f"動画が保存されました: {output_file}")
    #lidar_m_measurement_thread.join()
    #lidar_b_measurement_thread.join()
    cleanup(fd, urg, md)
    cv2.destroyAllWindows()
    cap.release()
    pygame.quit()

    ts = time.time()  # 現在のタイムスタンプ
    dt = datetime.fromtimestamp(ts)  # datetimeオブジェクトに変換
    TODAY_TIME = dt.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # ミリ秒を3桁に調整
    experiment_note_file.write(f"Finished Date: {TODAY_TIME}\n")
    experiment_note_file.close()

    sys.exit()
