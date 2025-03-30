import numpy as np
import cv2
import math
from scipy.optimize import differential_evolution, minimize
from scipy.ndimage import generic_filter
from scipy.ndimage import convolve

import time
import copy
import tkinter as tk

# import gridmap_update  #C++関数．使っていない

STORE_ROOT_DIR_NAME = f"slam_result_250330-2"
#enc_filepath = "./250314-2/enclog"  # enclogファイルのパス
#urg_filepath = "./250314-2/urglog"  # urglogファイルのパス

enc_filepath = "./250327-2/enclog"  # enclogファイルのパス
urg_filepath = "./250327-2/urglog"  # urglogファイルのパス

# エンコーダーデータの読み込み
def load_enclog(filepath):
    encdata = []
    with open(filepath, 'r') as file:
        for line in file:
            parts = line.strip().split(' ')
            timestamp = int(parts[0])
            x = float(parts[1])
            y = float(parts[2])
            a = float(parts[3])
            encdata.append((timestamp, x, y, a))
    return encdata

# LiDARデータの読み込み
def load_urglog(filepath):
    data = []
    with open(filepath, 'r') as file:
        for count, line in enumerate(file):
            parts = line.strip().split(' ')
            if len(parts) < 6:
                continue  # 必要なデータがない行はスキップ
            identifier = parts[0]  # 識別子
            timestamp = int(parts[1])  # タイムスタンプ
            data_count = int(parts[2])  # データ総数
            start_angle = float(parts[3])  # スタート角度r
            end_angle = float(parts[4])  # 終了角度
            angle_step = float(parts[5])  # 角度ステップ
            #try:
            #    ranges = [float(parts[7 + i]) for i in range(0, data_count, 4)]  # 3つごとに有効データを取得
            #except:
            #    print(parts)
            #    exit(0)
            ranges, intensity = zip(*[(float(parts[7 + i]), float(parts[7 + i + 3])) for i in range(0, data_count, 4)])
            #intensity = [1 for _ in range(1081)] 
            if count % 2 == 0:
                data.append((timestamp, start_angle, end_angle, angle_step, ranges, intensity))
    return data

def convert_lidar_to_points(start_angle, end_angle, angle_step, ranges, intensity, robot_pose):
    urg_shift_x = 0.165
    x, y, theta = robot_pose

    # ベクトル化とフィルタリング
    ranges = np.array(ranges)
    intensity = np.array(intensity)
    angles = np.radians(np.arange(start_angle, end_angle + angle_step, angle_step))
    valid_mask = (ranges > 300) & (ranges < 50000)
    ranges = ranges[valid_mask]
    intensity = intensity[valid_mask]
    angles = angles[valid_mask]

    # 位置計算のベクトル化
    ranges /= 1000
    cos_theta, sin_theta = np.cos(theta), np.sin(theta)
    lx = ranges * np.cos(angles) + urg_shift_x
    ly = ranges * np.sin(angles) 
    gx = x + lx * cos_theta - ly * sin_theta
    gy = y + lx * sin_theta + ly * cos_theta
    #data = np.column_stack((gx, gy, intensity))
    #for d in data:
    #    print(d)
    #exit()
    return np.column_stack((gx, gy, intensity))

def calc_relative_pose(prev, curr):
    _, px, py, pa = prev
    _, cx, cy, ca = curr
    dx = cx - px
    dy = cy - py
    da = ca - pa
    u = math.cos(pa) * dx + math.sin(pa) * dy
    v =-math.sin(pa) * dx + math.cos(pa) * dy
    a = da
    while True:
        if a > math.pi:
            a -= 2*math.pi
        elif a < -math.pi:
            a += 2*math.pi
        else:
            break
    return (prev[0], u, v, a)

# 共分散行列を計算
def fx(pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity):
    d = 0.001
    df = (eval_simple_func((pose[0] + d, pose[1], pose[2]), gridmap, start_angle, end_angle, angle_step, ranges, intensity) \
        - eval_simple_func(pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity))/d
    return df
    
def fy(pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity):
    d = 0.001
    df = (eval_simple_func((pose[0], pose[1] + d, pose[2]), gridmap, start_angle, end_angle, angle_step, ranges, intensity) \
        - eval_simple_func(pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity))/d
    return df

def fa(pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity):
    d = 0.001
    df = (eval_simple_func((pose[0], pose[1], pose[2] + d), gridmap, start_angle, end_angle, angle_step, ranges, intensity) \
        - eval_simple_func(pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity))/d
    return df

def fxx(pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity):
    d = 0.001
    df = (fx((pose[0] + d, pose[1], pose[2]), gridmap, start_angle, end_angle, angle_step, ranges, intensity) \
        - fx(pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity))/2/d
    return df

def fxy(pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity):
    d = 0.001
    df = (fx((pose[0], pose[1] + d, pose[2]), gridmap, start_angle, end_angle, angle_step, ranges, intensity) \
        - fx(pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity))/2/d
    return df
def fxa(pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity):
    d = 0.001
    df = (fx((pose[0], pose[1], pose[2] + d), gridmap, start_angle, end_angle, angle_step, ranges, intensity) \
        - fx(pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity))/2/d
    return df

def fyy(pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity):
    d = 0.001
    df = (fy((pose[0], pose[1] + d, pose[2]), gridmap, start_angle, end_angle, angle_step, ranges, intensity) \
        - fy(pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity))/2/d
    return df

def fya(pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity):
    d = 0.001
    df = (fy((pose[0], pose[1], pose[2] + d), gridmap, start_angle, end_angle, angle_step, ranges, intensity) \
        - fy(pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity))/2/d
    return df

def faa(pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity):
    d = 0.001
    df = (fa((pose[0], pose[1], pose[2] + d), gridmap, start_angle, end_angle, angle_step, ranges, intensity) \
        - fa(pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity))/2/d
    return df

def calc_Hmat(robot_pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity):
    eval_fxx = 1e-6*fxx(robot_pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity)
    eval_fxy = 1e-6*fxy(robot_pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity)
    eval_fxa = 1e-6*fxa(robot_pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity)
    eval_fyy = 1e-6*fyy(robot_pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity)
    eval_fya = 1e-6*fya(robot_pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity)
    eval_faa = 1e-6*faa(robot_pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity)
    print("-----<Covariance matrix>---")
    H = np.array([[eval_fxx, eval_fxy, eval_fxa],
                  [eval_fxy, eval_fyy, eval_fya],
                  [eval_fxa, eval_fya, eval_faa]])
    return H

def video_writer(output_file, gridmap):
    # 動画ライターの設定
    frame_height, frame_width, _ = gridmap.gmap.shape
    print(frame_height, frame_width)

    fps = 30  # フレームレート
    # MP4動画に保存準備
    fourcc = cv2.VideoWriter_fourcc(*'H264')  # コーデック指定
    return cv2.VideoWriter(output_file, fourcc, fps, (frame_height, frame_width))

def slam_process(enclog_data, urglog_data, gridmap, poses):
    # MP4動画に保存する処理
    video_out = video_writer(f"{STORE_ROOT_DIR_NAME}/slam_output.mp4", gridmap)

    time_diff_enc_urg = 50 # msec

    # lidarのtimestampと対応するenclogのデータを抽出しておく
    enclog_series = []
    for urgd in urglog_data:
        urg_timestamp, start_angle, end_angle, angle_step, ranges, intensity = urgd
        for encd in enclog_data:
            enc_timestamp, encx, ency, enca = encd
            if abs(urg_timestamp - enc_timestamp) < time_diff_enc_urg:  # タイムスタンプが一致するLiDARデータを選択
                enclog_series.append(encd)
                break
    # enclog_seriesを相対移動量に変換しておく
    prev_enc = enclog_series[0]
    relative_poses = []
    for enc in enclog_series:
        relative_pose = calc_relative_pose(prev_enc, enc)
        relative_poses.append(relative_pose)
        prev_enc = enc

    robot_poses = [[0, 0, 0, 0]]
    pu = 0.0
    pv = 0.0
    pa = 0.0
    for i in range(len(urglog_data)):
        urg_timestamp, start_angle, end_angle, angle_step, ranges, intensity = urglog_data[i]
        if len(ranges) != 1081: 
            print(f"skip data: {urg_timestamp} {i}")
            continue
        if i >= len(relative_poses): break
        enc_timestamp, u, v, a = relative_poses[i]
        _, rx, ry, ra = robot_poses[-1]
        rx = u*math.cos(ra) - v*math.sin(ra) + rx
        ry = u*math.sin(ra) + v*math.cos(ra) + ry
        ra = a + ra
        robot_pose = [rx, ry, ra]
        # robot_poseを初期値とし，最適化処理を経てrobot_poseを更新する
        if len(robot_poses) == 1:
            pass
        else:
            #print(".", end="", flush=True)
            s = time.time()
            robot_pose = optimize_pose_combined(gridmap, urglog_data[i], robot_pose)
            print(f"{time.time() - s:.3f} [s] {urg_timestamp} {i}")
        points = convert_lidar_to_points(start_angle, end_angle, angle_step, ranges, intensity, robot_pose)
        gridmap.update_gridmap(points)
        #gridmap.update_intensity(points, robot_pose)
        robot_poses.append([urg_timestamp, *robot_pose])

        if i % 100 == 0:
            img_disp = copy.deepcopy(gridmap.gmap)
            # 各ロボット座標に対して描画
            poses = np.array(robot_poses)  # robot_posesがリストの場合、NumPy配列に変換
            ix = (poses[:, 1] / gridmap.csize).astype(int) + gridmap.originX
            iy = (-poses[:, 2] / gridmap.csize).astype(int) + gridmap.originY
            # 画像描画
            for x, y in zip(ix, iy):
                cv2.circle(img_disp, (x, y), 5, (82, 54, 20), -1)


            # 各LiDAR計測点に対して描画
            img_disp_color = cv2.cvtColor(img_disp, cv2.COLOR_GRAY2BGR)
            for p in points:
                ix = int( p[0] / gridmap.csize) + gridmap.originX
                iy = int(-p[1] / gridmap.csize) + gridmap.originY
                cv2.circle(img_disp_color, (ix, iy), 5, (115, 224, 255), -1)

            rotated_img = cv2.rotate(img_disp_color, cv2.ROTATE_90_COUNTERCLOCKWISE)
            #rotated_img = cv2.cvtColor(rotated_img, cv2.COLOR_GRAY2BGR)
            video_out.write(rotated_img)
            cv2.imshow("gmap", img_disp_color)
            cv2.waitKey(5)

        ## ヘッセ行列とその逆行列を計算
        #H = calc_Hmat(robot_pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity)
        #try:
        #    H_inv = np.linalg.inv(H)
        #    print("ヘッセ行列の逆行列:")
        #    for row in H_inv:
        #        print(f"{row[0]:.4f} {row[1]:.4f} {row[2]:.4f}")
        #    print()
        #except np.linalg.LinAlgError:
        #    print("ヘッセ行列は逆行列を持ちません。")

    video_out.release()
    print(f"動画が保存されました")
    print("done")
    draw_poses_gmap = gridmap.draw_poses(robot_poses)
    cv2.imshow("gmap", draw_poses_gmap)
    with open(f"{STORE_ROOT_DIR_NAME}/cov_diagonal_elements.txt", "w") as f:
        for d in cov_diagonal_elements:
            f.write(f"{d[0]:.4f} {d[1]:.4f} {d[2]:.4f} {d[3]} {d[4]:.3f} {d[5]:.3f} {d[6]:.3f} {d[7]:.3f} {d[8]:.3f} {d[9]:.3f} {d[10]:.3f} {d[11]:.3f} {d[12]:.3f}\n")
    cv2.waitKey()

    return gridmap, robot_poses

# 評価関数をグローバルに移動
def eval_simple_func(pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity):
    sx, sy, sa = pose
    points = convert_lidar_to_points(start_angle, end_angle, angle_step, ranges, intensity, (sx, sy, sa))
    return gridmap.matches_simple(points)  # 評価値を反転（最小化のため）

def optimize_pose_combined(gridmap, urglog_data, robot_pose):
    urg_timestamp, start_angle, end_angle, angle_step, ranges, intensity = urglog_data
    dxy = 0.5
    da = 10*math.pi/180

    # 粗い探索
    bounds = [(robot_pose[0] - dxy, robot_pose[0] + dxy),
              (robot_pose[1] - dxy, robot_pose[1] + dxy),
              (robot_pose[2] - da, robot_pose[2] + da)]
    result_de = differential_evolution(
        eval_simple_func,
        bounds,
        args=(gridmap, start_angle, end_angle, angle_step, ranges, intensity),
        strategy='best1bin',
        popsize=20,     # ベイズ最適化で求めた86よりちょっと高くする
        tol=1e-6,
        maxiter=15,     # 同じく，38よりちょっと高くする
        workers=1,
        updating='deferred'  # 適応型の更新
    )
    return result_de.x

    # 精度悪い
    #result_da = dual_annealing(
    #    eval_simple_func, bounds, 
    #    args=(gridmap, start_angle, end_angle, angle_step, ranges, intensity),
    #      maxiter=30  # `differential_evolution` の maxiter に相当
    #)
    #return result_da.x

    ## 精密な探索
    #bounds = [(result_de.x[0] - 0.2, result_de.x[0] + 0.2),
    #          (result_de.x[1] - 0.2, result_de.x[1] + 0.2),
    #          (result_de.x[2] - 4 * math.pi/180, result_de.x[2] + 4 * math.pi/180)]
    #result = minimize(
    #    eval_simple_func,          
    #    result_de.x,                   # 差分進化法の結果を初期値として利用
    #    args=(gridmap, start_angle, end_angle, angle_step, ranges, intensity),
    #    method='CG',  
    #    #method='L-BFGS-B',             # 境界付き最適化
    #    #bounds=bounds,                 # 制約を考慮
    #    options={
    #        'disp': False,              # 結果を表示
    #        'maxiter': 500,            # 最大反復回数
    #        'ftol': 1e-8               # より高い精度を目指す
    #    }
    #)
    #return result.x

class Gridmap():
    def __init__(self, xmin, ymin, xmax, ymax, csize):
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax
        self.csize = csize
        self.width  = int((xmax - xmin)/csize)
        self.height = int((ymax - ymin)/csize)
        self.gmap = np.full((self.height, self.width, 1), 123, dtype=np.uint8)
        self.originX = int(abs(xmin)/csize)
        self.originY = int(abs(ymax)/csize)
        self.gmap_intensity = [[[] for _ in range(self.height)] for _ in range(self.width)]

    def update_gridmap(self, points):
        ix = ( points[:, 0] / self.csize).astype(int) + self.originX
        iy = (-points[:, 1] / self.csize).astype(int) + self.originY
        valid_mask = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
        ix = ix[valid_mask]
        iy = iy[valid_mask]
        self.gmap[iy, ix] = 0  # 一括更新

    def update_intensity(self, points, pose):
        ix = ( points[:, 0] / self.csize).astype(int) + self.originX
        iy = (-points[:, 1] / self.csize).astype(int) + self.originY
        intensity = points[:,2]
        valid_mask = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
        ix = ix[valid_mask]
        iy = iy[valid_mask]
        intensity = intensity[valid_mask]
        for iix, iiy, iii in zip(ix, iy, intensity):
            # r: 計測距離
            # nx, ny: 規格化した方向ベクトル
            # poseはその反射強度を計測したときのロボットの位置
            r = np.linalg.norm([pose[0], pose[1]])
            nx, ny = pose[0:2]/r
            intensity_data = (iii, pose[0], pose[1], r, nx, ny)    
            # 対応するグリッドマップ上のリストにデータを追加
            #self.gmap_intensity[iiy][iix].append(intensity_data)  
            if 0 <= iiy < len(self.gmap_intensity) and 0 <= iix < len(self.gmap_intensity[iiy]):
                self.gmap_intensity[iiy][iix].append(intensity_data)
            else:
                print(f"範囲外のアクセス: iiy={iiy}, iix={iix}")

    def draw_poses(self, poses):
        gmap = copy.deepcopy(self.gmap)
        for _, px, py, pa in poses:
            ix = int( px/self.csize) + self.originX
            iy = int(-py/self.csize) + self.originY
            cv2.circle(gmap, (ix, iy), 1, (255, 255, 255), -1, cv2.LINE_AA)
        return gmap

    def matches_simple(self, points):
        # NumPy配列に変換
        points = np.array(points)
        # x, y 座標の変換をベクトル化
        ix = ( points[:, 0] / self.csize).astype(int) + self.originX
        iy = (-points[:, 1] / self.csize).astype(int) + self.originY
        # 範囲内にあるインデックスをフィルタリング
        valid = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
        # 有効なインデックスのみを抽出
        ix_valid = ix[valid]
        iy_valid = iy[valid]
        # マップの値が 0 の場合のみカウント
        eval = np.sum(self.gmap[iy_valid, ix_valid] == 0)
        #eval = np.sum(generic_filter(self.gmap, self.check_region, size=3, mode='constant', cval=1))

        return -eval
    
    def gridmap_clear(self):
        self.gmap = np.full((self.height, self.width, 1), 123, dtype=np.uint8)

def init_gridmap(xmin = -5.0, ymin = -5.0, xmax = 5.0, ymax = 5.0, csize = 0.05):
    gmap = Gridmap(xmin, ymin, xmax, ymax, csize)
    return gmap
    
# メイン処理
from pathlib import Path # pathの操作
import readchar # 1文字の入力受付
import os
import shutil
if __name__ == "__main__":
    if Path(STORE_ROOT_DIR_NAME).exists():
        print(f"'{STORE_ROOT_DIR_NAME}' exists.")
        print("Do you want to continue? y/[n] ", end="", flush=True)
        response = readchar.readchar()  # 1文字入力を受け付ける
        if response.lower() == "y":
            print("\nContinuing...")
            print("Delete existing log files")
            files_to_delete = ["robot_poses.txt", "slam_output.mp4", "opt.png", "mapinfo.lua"]
            for file in files_to_delete:
                file_path = os.path.join(STORE_ROOT_DIR_NAME, file)
                if os.path.exists(file_path):
                    os.remove(file_path)
                    print(f"Deleted: {file_path}")
            print("✅ Delete existing log files")
            # 必要なファイルが存在するか確認
            if os.path.isfile(os.path.join(STORE_ROOT_DIR_NAME, "enclog")):
                print("✅ exist enclog")
            else:
                print("enclogを用意してください．")
                exit()
            if os.path.isfile(os.path.join(STORE_ROOT_DIR_NAME, "urglog")):
                print("✅ exist urglog")
            else:
                print("urglogを用意してください．")
                exit()
        else:
            print("\nExiting...")
            exit()
    else:
        print(f"'{STORE_ROOT_DIR_NAME}' does not exist.")
        # ディレクトリがなければ作成
        os.makedirs(STORE_ROOT_DIR_NAME, exist_ok=True)
        print("enclog, urglogを用意します")
        shutil.copy(enc_filepath, f"{STORE_ROOT_DIR_NAME}/")
        shutil.copy(urg_filepath, f"{STORE_ROOT_DIR_NAME}/")
        if os.path.isfile(os.path.join(STORE_ROOT_DIR_NAME, "enclog")):
            print("✅ exist enclog")
        else:
            print("enclogを用意してください．")
            exit()
        if os.path.isfile(os.path.join(STORE_ROOT_DIR_NAME, "urglog")):
            print("✅ exist urglog")
        else:
            print("urglogを用意してください．")
            exit()

    # データの読み込み
    enclog_data = load_enclog(enc_filepath)  
    urglog_data = load_urglog(urg_filepath)

    # D棟周回
    if 0:
        minX = -25.0
        minY = -100.0
        maxX =  80.0
        maxY =  25.0
        csize = 0.025

    #minX = -15.0
    #minY = -15.0
    #maxX =  15.0
    #maxY =  15.0
    #csize = 0.025

    #minX = -5.0
    #minY = -15.0
    #maxX = 75.0
    #maxY = 15.0
    #csize = 0.025

    # 体育館〜D棟周回
    minX = -75.0
    minY = -160.0
    maxX = 75.0
    maxY = 35.0
    csize = 0.025

    # gridmap を初期化
    #gridmap = init_gridmap(xmin = -35.0, ymin = -20.0, xmax = 35.0, ymax = 55.0, csize = 0.025)
    gridmap = init_gridmap(minX, minY, maxX, maxY, csize)

    with open(f"{STORE_ROOT_DIR_NAME}/mapInfo.lua", "w") as file:
        file.write("local mapInfo = {\n")
        file.write(f"\toriginX = {gridmap.originX},\n")
        file.write(f"\toriginY = {gridmap.originY},\n")
        file.write(f"\tcsize = {csize},\n")
        file.write(f"\tminX = {minX},\n")
        file.write(f"\tminY = {minY},\n")
        file.write(f"\tmaxX = {maxX},\n")
        file.write(f"\tmaxY = {maxY},\n")
        file.write(f"\tmargin = 50,\n")
        file.write("}\n")
        file.write("return mapInfo\n")

    # SLAM処理を実行して最適化されたマップデータを取得
    s = time.time()
    poses = [(0, 0, 0)] # SLAM開始時点の初期姿勢
    gridmap, poses = slam_process(enclog_data, urglog_data, gridmap, poses)
    print(f"Executed Time: {time.time()-s}")

    cv2.imshow("gmap", gridmap.gmap)
    cv2.imwrite(f"{STORE_ROOT_DIR_NAME}/opt.png", gridmap.gmap)
    with open(f"{STORE_ROOT_DIR_NAME}/robot_poses.txt", "w") as file:
        for p in poses:
            file.write(f"{p[0]} {p[1]} {p[2]} {p[3]}\n")
    cv2.waitKey()
    
    # 全ての処理が完了したら，intensity_grid_map を可視化する
    #intensity_grid_map = np.zeros_like(gridmap.gmap)
    #intensity_grid_map = cv2.cvtColor(intensity_grid_map, cv2.COLOR_GRAY2BGR)
    #log_intensity = []
    #for row in range(gridmap.height):
    #    if row >= len(gridmap.gmap_intensity):
    #        continue
    #    for col in range(gridmap.width):
    #        if col >= len(gridmap.gmap_intensity[row]):
    #            continue
    #        if len(gridmap.gmap_intensity[row][col]) > 0:
    #            # 占有点座標をワールド座標系へ変換
    #            px = (col - gridmap.originX) * gridmap.csize 
    #            py =-(row - gridmap.originX) * gridmap.csize 
    #            # 反射強度ベクトル
    #            vx = 0
    #            vy = 0
    #            max_intensity_leng = 0
    #            log = []
    #            for data in gridmap.gmap_intensity[row][col]:
    #                intensity = int(data[0])
    #                leng = intensity // 200
    #                max_intensity_leng = max(max_intensity_leng, leng)
    #                rx = data[1]
    #                ry = data[2]
    #                dx = px - rx
    #                dy = py - ry
    #                #norm = math.sqrt(dx**2 + dy**2)
    #                norm = np.linalg.norm([dx, dy])
    #                dx /= norm
    #                dy /= norm
    #                vx += dx
    #                vy += dy
    #                # row, col の場所（格子）ごとに，反射強度と方向でヒストグラムを作成する
    #                # 作成したヒストグラムを用いて，循環ガウス分布でフィッティングをする
    #                th = math.atan2(dy, dx)
    #                log.append((th, intensity))
    #            log_intensity.append([row, col, log])
    #            vx /= len(gridmap.gmap_intensity[row][col])
    #            vy /= len(gridmap.gmap_intensity[row][col])
    #            th = math.atan2(vy, vx)
    #            cv2.line(intensity_grid_map, (col, row), 
    #                     (col + int(max_intensity_leng*math.cos(th)), row - int(max_intensity_leng*math.sin(th))),
    #                     (255, 0, 0), 1, cv2.LINE_AA)
    #            cv2.circle(intensity_grid_map, (col, row), 1, (255, 255, 255), -1, cv2.LINE_AA)

    # 計測データが一定数以上のグリッドを抽出する
    #selected_data = [cell for cell in log_intensity if len(cell[2]) > 100]
    # selected_dataの構成
    # (row, col, log[])
    # - row, col: 格子地図の行，列座標
    # - log[]: 計測データの集合．各データは以下のタプルで保持する
    #          (intensity, pose_x, pose_y, r, nx, ny)
    # - intensity: 反射強度
    # - r: 計測距離
    # - nx, ny: 規格化した方向ベクトル
    # - pose_x, pose_y: その反射強度を計測したときのロボットの位置座標
    #   したがって，r = norm(pose_x, pose_y)
    #import matplotlib.pyplot as plt
    #for cell in selected_data:
    #    x = []
    #    y = []
    #    print(cell[0], cell[1])
    #    for th, intensity in cell[2]:
    #        x.append(th)
    #        y.append(intensity)
    #    
    #    sorted_pairs = sorted(zip(x, y), key=lambda pair: pair[0])
    #    x_sorted, y_sorted = zip(*sorted_pairs)
    #    plt.plot(x_sorted, y_sorted)#, s=0.5)
    #    plt.xlim(-np.pi, np.pi)
    #    plt.ylim(0, 15000)
    #    plt.grid()
    #    plt.show()
    #exit()
    
    ## グレースケールに変換
    #gray = cv2.cvtColor(intensity_grid_map, cv2.COLOR_BGR2GRAY)
    ## 背景（黒）を基準に図形を囲むバウンディングボックスを計算
    #_, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)  # 黒背景以外を白にする
    #x, y, w, h = cv2.boundingRect(thresh)  # バウンディングボックスを取得
    
    ## バウンディングボックスで画像をトリミング
    #trimmed_image = intensity_grid_map[y:y+h, x:x+w]
    
    ## 拡大倍率 (例: 2倍)
    #scale = 4
    #new_size = (int(trimmed_image.shape[1] * scale), int(trimmed_image.shape[0] * scale))
    ## 画像を拡大
    #resized_image = cv2.resize(trimmed_image, new_size, interpolation=cv2.INTER_NEAREST)
    #cv2.imshow("intensity_grid_map", resized_image)
    #cv2.waitKey()
