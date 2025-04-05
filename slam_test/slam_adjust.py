import numpy as np
import cv2
import math
import sys
from scipy.optimize import differential_evolution, minimize
import PyQtImageWriter

import time
import copy

STORE_ROOT_DIR_NAME = f"slam_result_250404-1"
enc_filepath = f"{STORE_ROOT_DIR_NAME}/enclog"  # enclogファイルのパス
urg_filepath = f"{STORE_ROOT_DIR_NAME}/urglog"  # urglogファイルのパス
robot_poses_filepath = f"{STORE_ROOT_DIR_NAME}/robot_poses.txt"

# PyQtの準備
app = PyQtImageWriter.init()
window = PyQtImageWriter.ImageWindow()
window.show()

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

def load_robot_poses(filepath):
    robot_poses = []
    with open(filepath, 'r') as file:
        for line in file:
            parts = line.strip().split(' ')
            timestamp = int(parts[0])
            x = float(parts[1])
            y = float(parts[2])
            a = float(parts[3])
            robot_poses.append([timestamp, x, y, a])
    return robot_poses

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
    de_cov_matrix_list = []
    cov_diagonal_elements = []
    for i in range(len(urglog_data)):
        start_time = time.time()
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
            print(f"optimized {time.time() - s:.3f} [s] {urg_timestamp} {i}")
            searched_pose_list_np = np.array(searched_pose_list)
            de_cov_matrix = np.cov(searched_pose_list_np[:, :3], rowvar=False)
            de_cov_matrix_list.append(de_cov_matrix)
            #print(f"{de_cov_matrix[0,0]:.2f} {de_cov_matrix[1,1]:.2f} {de_cov_matrix[2,2]:.2f}")
            #print(f"{de_cov_matrix[0,0]:.2f} {de_cov_matrix[1,1]:.2f} {de_cov_matrix[2,2]:.2f}")
            # 今回のオドメトリによる相対姿勢u, v, a
            # 今回のDEによる相対姿勢
            # robot_pose - robot_poses[-1][1:4]
            rp = calc_relative_pose(robot_poses[-1], [0, *robot_pose])
            #print(u, v, a)
            #print(rp[1:4])
            cov_diagonal_elements.append([de_cov_matrix[0,0], de_cov_matrix[1,1], de_cov_matrix[2,2], urg_timestamp, *robot_pose, 
                                         u, v, a, rp[1], rp[2], rp[3]])
            # 推定した角度変化が10deg以上の場合，ジャンプしすぎていると判断し，オドメトリを使って姿勢更新
            if abs(rp[3]) > 0.17:
                #print("Corrected pose to odometory.")
                #print(u, v, a, rp[1], rp[2], rp[3])
                robot_pose = [rx, ry, ra]
            
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

        print(f"Loop time:{time.time() - start_time:.3f}")
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

def slam_process_re(robot_poses, urglog_data, gridmap):
    global draw_img
    # すでに得られた地図を手動調整する

    time_diff_robotpose_urg = 50 # msec

    # robot_posesrのtimestampと対応するurglogのデータを抽出しておく
    urglog_series = []
    for pose in robot_poses:
        ts, _, _, _ = pose
        for urgd in urglog_data:
            urg_timestamp, start_angle, end_angle, angle_step, ranges, intensity = urgd
            if abs(urg_timestamp - ts) < time_diff_robotpose_urg:  # タイムスタンプが一致するLiDARデータを選択
                urglog_series.append(urgd)
                break
    # robot_posesを相対移動量に変換しておく
    prev_pose = robot_poses[0]
    relative_poses = []
    for ind, pose in enumerate(robot_poses):
        if ind == 0:
            continue
        relative_pose = calc_relative_pose(prev_pose, pose)
        relative_poses.append(relative_pose)
        prev_pose = pose

    # Gridmapを描いてみる
    for ind, robot_pose in enumerate(robot_poses):
        urg_timestamp, start_angle, end_angle, angle_step, ranges, intensity = urglog_series[ind]
        points = convert_lidar_to_points(start_angle, end_angle, angle_step, ranges, intensity, robot_pose[1:])
        gridmap.update_gridmap(points)

    # robot_posesを描く
    img_disp = copy.deepcopy(gridmap.gmap)
    img_disp = cv2.cvtColor(img_disp, cv2.COLOR_GRAY2BGR)

    for p in robot_poses:
        ix = int( p[1] / gridmap.csize) + gridmap.originX
        iy = int(-p[2] / gridmap.csize) + gridmap.originY
        cv2.circle(img_disp, (ix, iy), 5, (82, 54, 20), -1)

    cur_pose = robot_poses[0]
    cur_index = 0
    print("編集点を選択します．[n]ext, [p]revious, [s]elect")
    img_disp_tmp = copy.deepcopy(img_disp)
    key = None
    while True:
        if PyQtImageWriter.pressed_key == ord('q'):
            break
        elif PyQtImageWriter.pressed_key == None:
            pass
        elif PyQtImageWriter.pressed_key == ord('n'):
            cur_index += 1
            if cur_index >= len(robot_poses):
                cur_index = 0   # 最初に戻る
        elif PyQtImageWriter.pressed_key == ord('N'):
            cur_index += 10
            if cur_index >= len(robot_poses):
                cur_index = 0   # 最初に戻る
        elif PyQtImageWriter.pressed_key == ord('p'):
            cur_index -= 1
            if cur_index < 0:
                cur_index = len(robot_poses) - 1    # 最後に進む
        elif PyQtImageWriter.pressed_key == ord('P'):
            cur_index -= 10
            if cur_index < 0:
                cur_index = len(robot_poses) - 1    # 最後に進む
        elif PyQtImageWriter.pressed_key == ord('s'):
            print("select the current pose")
            print("hjkl: move the robot in the xy direction / ui: rotate the robot ccw or cw")
            while True:
                if PyQtImageWriter.pressed_key == ord('h'):
                    robot_poses[cur_index][1] -= 5*gridmap.csize
                elif PyQtImageWriter.pressed_key == ord('j'):
                    robot_poses[cur_index][2] -= 5*gridmap.csize
                elif PyQtImageWriter.pressed_key == ord('k'):
                    robot_poses[cur_index][2] += 5*gridmap.csize
                elif PyQtImageWriter.pressed_key == ord('l'):
                    robot_poses[cur_index][1] += 5*gridmap.csize
                elif PyQtImageWriter.pressed_key == ord('u'):
                    print("pressed u")
                    robot_poses[cur_index][3] += 0.1*np.pi/180
                elif PyQtImageWriter.pressed_key == ord('i'):
                    print("pressed i")
                    robot_poses[cur_index][3] -= 0.1*np.pi/180
                elif PyQtImageWriter.pressed_key == ord('q'):
                    print("pressed q")
                    break
                # 移動したcur_indexとその前の相対移動量を更新する
                if cur_index == 0:
                    continue
                prev_pose = robot_poses[cur_index-1]
                relative_pose = calc_relative_pose(prev_pose, robot_poses[cur_index])
                relative_poses[cur_index-1] = relative_pose
                ## 新しいrelative_posesでrobot_poses全体を更新する
                prev_pose = robot_poses[0]
                new_robot_poses = []
                new_robot_poses.append(prev_pose)
                for ind in range(len(robot_poses)-1):
                    _, u, v, a = relative_poses[ind]
                    _, px, py, pa = prev_pose
                    x = math.cos(pa) * u - math.sin(pa) * v + px
                    y = math.sin(pa) * u + math.cos(pa) * v + py
                    a = a + pa
                    new_robot_poses.append([robot_poses[ind][0], x, y, a])
                    prev_pose = [robot_poses[ind][0], x, y, a]
                robot_poses = copy.deepcopy(new_robot_poses)
                # 新しいgridmapを用意する
                gridmap.gridmap_clear()
                # 基準マップを配置する
                #gridmap.base_map_draw()
                # 新しく点群全体を配置する
                for ind, robot_pose in enumerate(robot_poses):
                    urg_timestamp, start_angle, end_angle, angle_step, ranges, intensity = urglog_series[ind]
                    points = convert_lidar_to_points(start_angle, end_angle, angle_step, ranges, intensity, robot_pose[1:])
                    gridmap.update_gridmap(points)
                img_disp = gridmap.gmap
                # robot_posesを描く
                img_disp_tmp = copy.deepcopy(img_disp)
                img_disp_tmp = cv2.cvtColor(img_disp_tmp, cv2.COLOR_GRAY2BGR)
                #for p in robot_poses:
                #    ix = int( p[1] / gridmap.csize) + gridmap.originX
                #    iy = int(-p[2] / gridmap.csize) + gridmap.originY
                #    cv2.circle(img_disp_tmp, (ix, iy), 5, (82, 54, 20), -1)
                poses = np.array(robot_poses)  # (N, 3) の形に変換
                ix = (poses[:, 1] / gridmap.csize).astype(int) + gridmap.originX
                iy = (-poses[:, 2] / gridmap.csize).astype(int) + gridmap.originY
                # OpenCVのcircle関数はループが必要なので、リスト内包表記を活用
                [cv2.circle(img_disp_tmp, (x, y), 5, (82, 54, 20), -1) for x, y in zip(ix, iy)]
                draw_img = img_disp_tmp
                frame = copy.deepcopy(draw_img)
                window.set_frame(frame)
            # cur_indexの場所からSLAMを実行するかどうか
            print("Retry SLAM current index? y/[n]")
            while True:
                if PyQtImageWriter.pressed_key == ord('y'):
                    print("Execute SLAM")
                    # cur_indexまでの経路でグリッドマップを作成
                    # 新しいgridmapを用意する
                    gridmap.gridmap_clear()
                    # 基準マップを配置する
                    #gridmap.base_map_draw()
                    # 新しく点群全体を配置する
                    for ind, robot_pose in enumerate(robot_poses):
                        if ind < cur_index: # cur_indexまでの点群だけを描画する．
                            urg_timestamp, start_angle, end_angle, angle_step, ranges, intensity = urglog_series[ind]
                            points = convert_lidar_to_points(start_angle, end_angle, angle_step, ranges, intensity, robot_pose[1:])
                            gridmap.update_gridmap(points)
                    # ここからがcur_indexから始めるSLAM
                    for i in range(cur_index, len(urglog_data)):
                        urg_timestamp, start_angle, end_angle, angle_step, ranges, intensity = urglog_data[i]
                        if len(ranges) != 1081: 
                            print(f"skip data: {urg_timestamp} {i}")
                            continue
                        if i >= len(relative_poses): break
                        _, u, v, a = relative_poses[i]
                        _, rx, ry, ra = robot_poses[i-1]  # このindexはSLAMで推定した最後の姿勢．要確認
                        rx = u*math.cos(ra) - v*math.sin(ra) + rx
                        ry = u*math.sin(ra) + v*math.cos(ra) + ry
                        ra = a + ra
                        robot_pose = [rx, ry, ra]
                        # robot_poseを初期値とし，最適化処理を経てrobot_poseを更新する
                        if len(robot_poses) == 1:
                            pass
                        else:
                            robot_pose = optimize_pose_combined(gridmap, urglog_data[i], robot_pose)
                        points = convert_lidar_to_points(start_angle, end_angle, angle_step, ranges, intensity, robot_pose)
                        gridmap.update_gridmap(points)
                        #gridmap.update_intensity(points, robot_pose)
                        robot_poses[i]=[urg_timestamp, robot_pose[0], robot_pose[1], robot_pose[2]]
                        print(f"{robot_poses[i][0]} {robot_poses[i][1]:.3f} {robot_poses[i][1]:.3f} {robot_poses[i][1]*180/np.pi:.1f}")

                    img_disp = gridmap.gmap
                    # robot_posesを描く
                    img_disp_tmp = copy.deepcopy(img_disp)
                    img_disp_tmp = cv2.cvtColor(img_disp_tmp, cv2.COLOR_GRAY2BGR)
                    poses = np.array(robot_poses)  # (N, 3) の形に変換
                    ix = (poses[:, 1] / gridmap.csize).astype(int) + gridmap.originX
                    iy = (-poses[:, 2] / gridmap.csize).astype(int) + gridmap.originY
                    # OpenCVのcircle関数はループが必要なので、リスト内包表記を活用
                    [cv2.circle(img_disp_tmp, (x, y), 5, (82, 54, 20), -1) for x, y in zip(ix, iy)]
                    draw_img = img_disp_tmp
                    frame = copy.deepcopy(draw_img)
                    window.set_frame(frame)
                    break
                elif PyQtImageWriter.pressed_key == ord('n'):
                    print("Exit selection mode")
                    break
                window.set_frame(frame)
        s = time.time()
        for p in robot_poses:
            ix = int( p[1] / gridmap.csize) + gridmap.originX
            iy = int(-p[2] / gridmap.csize) + gridmap.originY
            cv2.circle(img_disp_tmp, (ix, iy), 5, (82, 54, 20), -1)
        print(f"draw robot_poses: {time.time() - s}")
        p = robot_poses[cur_index]
        ix = int( p[1] / gridmap.csize) + gridmap.originX
        iy = int(-p[2] / gridmap.csize) + gridmap.originY
        cv2.circle(img_disp_tmp, (ix, iy), 20, (200, 200, 0), -1)

        frame = copy.deepcopy(img_disp_tmp)
        #frame = cv2.imread("250314-3/rebuild_opt.png")
        # PyQtの準備
        window.set_frame(frame)

    # 結果の書き出し
    draw_poses_gmap = gridmap.draw_poses(robot_poses)
    cv2.imshow("gmap", draw_poses_gmap)
    cv2.imwrite("rebuild_opt.png", gridmap.gmap)
    cv2.waitKey()

    # 再構築したロボット軌跡をファイルに書き出す
    with open("rebuild_robot_poses.txt", "w") as file:
        for p in robot_poses:
            file.write(f"{p[0]} {p[1]} {p[2]} {p[3]}\n")

    print("Completed.")
    # PyQtの描画を終了
    # この処理はプロセス自体を止める．以下のprint文は実行されないので注意
    # returnもしない
    sys.exit(app.exec_())
    print("bye")
    return gridmap, robot_poses

# 評価関数をグローバルに移動
searched_pose_list = []
def eval_simple_func(pose, gridmap, start_angle, end_angle, angle_step, ranges, intensity):
    global searched_pose_list
    sx, sy, sa = pose
    points = convert_lidar_to_points(start_angle, end_angle, angle_step, ranges, intensity, (sx, sy, sa))
    N = min(500, len(points)) # 抽出する点の数
    indices = np.random.choice(points.shape[0], N, replace=False)  # 重複なし
    points = points[indices]
    eval = gridmap.matches_simple(points)  # 評価値を反転（最小化のため）
    searched_pose_list.append([sx, sy, sa, eval])
    return eval

#from scipy.optimize import dual_annealing
def optimize_pose_combined(gridmap, urglog_data, robot_pose):
    global searched_pose_list
    searched_pose_list = []
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
        # 以下は基準マップの再描画に使用する．resize時に登録する
        self.base_gmap = None
        self.base_gmap_width = None
        self.base_gmap_height = None
        self.base_gmap_left_x = None
        self.base_gmap_left_y = None
        self.base_gmap_right_x = None
        self.base_gmap_right_y = None

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
            if ix >= 0 and ix < self.width and iy >= 0 and iy < self.height:
                gmap[iy-1, ix  ] = 255
                gmap[iy  , ix-1] = 255
                gmap[iy  , ix  ] = 255
                gmap[iy  , ix+1] = 255
                gmap[iy+1, ix  ] = 255
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

    def resize(self, new_minX, new_minY, new_maxX, new_maxY):
        new_width  = int((new_maxX - new_minX)/csize)
        new_height = int((new_maxY - new_minY)/csize)
        self.base_gmap = copy.deepcopy(self.gmap)
        old_gmap = self.base_gmap[:, :, np.newaxis]  # (H, W) → (H, W, 1)
        h, w, _ = old_gmap.shape

        self.gmap = np.full((new_height, new_width, 1), 223, dtype=np.uint8)
        # 既存のgmapを新しいgmapに挿入
        # img_integrated[0:height, width_img_disp_color:width_img_disp_color+width] = frame
        left_x = int(abs(new_minX - self.xmin)/self.csize)
        left_y = int(abs(new_maxY - self.ymax)/self.csize)
        right_x = left_x + self.width
        right_y = left_y + self.height
        print(old_gmap.shape)
        print(self.gmap.shape)
        print(self.height, self.width)
        print(left_x, left_y, right_x, right_y)
        self.gmap[left_y:right_y, left_x:right_x] = old_gmap

        self.xmin = new_minX
        self.ymin = new_minY
        self.xmax = new_maxX
        self.ymax = new_maxY
        self.width  = int((self.xmax - self.xmin)/self.csize)
        self.height = int((self.ymax - self.ymin)/self.csize)
        self.originX = int(abs(self.xmin)/self.csize)
        self.originY = int(abs(self.ymax)/self.csize)

        self.base_gmap_width = w
        self.base_gmap_height = h
        self.base_gmap_left_x = left_x
        self.base_gmap_left_y = left_y
        self.base_gmap_right_x = right_x
        self.base_gmap_right_y = right_y

    def base_map_draw(self):
        old_gmap = self.base_gmap[:, :, np.newaxis]  # (H, W) → (H, W, 1)
        self.gmap[self.base_gmap_left_y:self.base_gmap_right_y, self.base_gmap_left_x:self.base_gmap_right_x] = old_gmap
        
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
        pass
    else:
        print(f"'{STORE_ROOT_DIR_NAME}' does not exist.")
        exit()

    # データの読み込み
    enclog_data = load_enclog(enc_filepath)  
    urglog_data = load_urglog(urg_filepath)
    robot_poses = load_robot_poses(robot_poses_filepath)

    # 体育館〜D棟周回
    minX = -75.0
    minY = -160.0
    maxX = 75.0
    maxY = 35.0
    csize = 0.025
    # 図書館前~体育館
    minX = -75.0
    minY = -160.0
    maxX = 170.0
    maxY = 35.0
    csize = 0.025

    # gridmap を初期化
    gridmap = init_gridmap(minX, minY, maxX, maxY, csize)

    # SLAM処理を実行して最適化されたマップデータを取得
    # 地図の手動調整（SLAMを実行して必要なファイルが存在すること）
    gridmap, poses = slam_process_re(robot_poses, urglog_data, gridmap)
