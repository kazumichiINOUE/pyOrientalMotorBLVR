import cv2
import numpy as np
from math import pi, acos
import time

# 色指定の形式変換
def hex_to_rgb(hex_color):
    hex_color = hex_color.lstrip('#')
    return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))

def xy2index(xd, yd, csize, ox, oy):
    ix = np.int32(xd / csize + ox)
    iy = np.int32(-yd / csize + oy)
    return ix, iy

def update_map(map, scan_px, scan_py, current_x, current_y, current_a, width, height, ox, oy, csize):
    cs, sn = np.cos(current_a), np.sin(current_a)
    xd = scan_px * cs - scan_py * sn + current_x
    yd = scan_px * sn + scan_py * cs + current_y
    ix, iy = xy2index(xd, yd, csize, ox, oy)
    mask = (0 <= ix) & (ix < width) & (0 <= iy) & (iy < height)
    map[iy[mask], ix[mask]] = 1
    return map

def gmap_show(map, width, height, waitTime = 0):
    img_org = np.zeros((height, width, 3), dtype=np.uint8)
    img_org[:] = hex_to_rgb('#e6e7ed')  # 背景色
    img_org[map == 1] = hex_to_rgb('#33635c')
    # TODO test
    cv2.imshow("slam", img_org)
    cv2.waitKey(waitTime)

def gmap_save(map, width, height, fname = "result/result.png"):
    img_org = np.zeros((height, width, 3), dtype=np.uint8)
    img_org[:] = hex_to_rgb('#e6e7ed')  # 背景色
    img_org[map == 1] = hex_to_rgb('#33635c')
    cv2.imwrite(fname, img_org)

def transf(scan_px, scan_py, cx, cy, cs, sn):
    xd = scan_px * cs - scan_py * sn + cx
    yd = scan_px * sn + scan_py * cs + cy
    return xd, yd

def main():
    width = 2000  # グリッドマップのサイズ
    height = 1000
    ox = width/4
    oy = height/4 * 3
    csize = 0.025  # [m] 格子の解像度
    gmap = np.zeros((height, width), dtype=np.uint8)
    
    scan_px = np.zeros(1081)
    scan_py = np.zeros(1081)
    
    current_x = 0
    current_y = 0
    current_a = 0
    
    #count = 0
    deg2pi = pi/180
    
    path = []
    fname = f"./urglog"
    with open(fname, "r") as file:
        #best_x, best_y, best_a = 0, 0, 0
        for ind, line in enumerate(file):
            data = line.split()
            ts = data[1]
            data_size = int(data[2])
            start_angle = float(data[3])
            #end_angle = float(data[4])
            step_angle = float(data[5])
            echo_size = int(data[6])
            data_size = data_size//echo_size
            for i in range(0, data_size):
                r = float(float(data[7 + i * echo_size]) / 1000)
                th = (start_angle + step_angle * i)*deg2pi
                scan_px[i] = r * np.cos(th)
                scan_py[i] = r * np.sin(th)
            if ind == 0:
                gmap = update_map(gmap, scan_px, scan_py, current_x, current_y, current_a, width, height, ox, oy, csize)
                path.append([ts, current_x, current_y, current_a])
                continue
    
            start_time = time.time()
            # 現在姿勢を中心とした探索窓をnp.meshgrid()で作成する
            ddx = np.arange(-0.5 + current_x, 0.5 + current_x, csize)
            ddy = np.arange(-0.5 + current_y, 0.5 + current_y, csize)
            max_length = 15 # 回転方向を探索する際の1ピクセルを超えないことを保証する距離
                            # 長い方が処理時間は伸びる(delta_thが細かくなるため)
            delta_th = acos(1 - csize**2/max_length**2)
            dth = np.arange(-pi/4 + current_a, pi/4 + current_a, delta_th)
            cx, cy, ca = np.meshgrid(ddx, ddy, dth)
            cs = np.cos(ca)
            sn = np.sin(ca)
            
            eval_matrix = np.zeros_like(cx)     # 探索窓内での候補点に対する評価値
            for i in range(0, len(scan_px), 32):    # スキャン点に対してループを作成する
                                                    # この処理をベクトル化しても高速にはならなかった
                xd, yd = transf(scan_px[i], scan_py[i], cx, cy, cs, sn)
                ix, iy = xy2index(xd, yd, csize, ox, oy)
                mask = (0 <= ix) & (ix < width) & (0 <= iy) & (iy < height)
                eval_matrix[mask] += gmap[iy[mask], ix[mask]]
            
            best_index = np.unravel_index(np.argmax(eval_matrix), eval_matrix.shape)
            current_x, current_y, current_a = cx[best_index], cy[best_index], ca[best_index]
            
            end_time = time.time()
            print(f" {end_time - start_time:.2f}[s]", end=" ")
            gmap = update_map(gmap, scan_px, scan_py, current_x, current_y, current_a, width, height, ox, oy, csize)
            path.append([ts, current_x, current_y, current_a])
            print(f"{current_x:.3f} {current_y:.3f} {current_a*180/pi:.1f}")
            gmap_show(gmap, width, height, 10)
    
    # 推定した姿勢の書き出し icpSlamの処理に合わせたフォーマットにしてある
    with open("./result/path", "w") as file:
        for p in path:
            file.write(f"{p[0]} {p[1]} {p[2]} {p[3]} 0.0 0.0 0.0 0.0 0.0 0 1 1\n")
        print("path store done.")
    gmap_save(gmap, width, height)
    print("result.png store done.")
    gmap_show(gmap, width, height)

main()
