import cv2
import numpy as np
import copy
import math
from Colors import hex_to_rgb
import ReadConfig as rc
# Include default configuration
config = rc.read_config('config.lua')

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

def gmap_show(map, width, height, path, waitTime = 0):
    img_org = np.zeros((height, width, 3), dtype=np.uint8)
    img_org[:] = hex_to_rgb('#e6e7ed')  # 背景色
    img_org[map == 1] = hex_to_rgb('#33635c')
    cv2.imshow("slam", img_org)
    cv2.waitKey(waitTime)

def transf(scan_px, scan_py, cx, cy, cs, sn):
    xd = scan_px * cs - scan_py * sn + cx
    yd = scan_px * sn + scan_py * cs + cy
    return xd, yd

class Slam:
    """
    """
    def __init__(self):
        self.count = 0
        self.csize = config.slam.csize
        self.width = config.slam.window_width
        self.height = config.slam.window_height
        self.ox = config.slam.origin_x
        self.oy = config.slam.origin_y

        #self.img_org = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        #self.img_org[:] = hex_to_rgb(config.map.color.bg)
        #cv2.line(self.img_org, (0, self.oy), (self.width, self.oy), hex_to_rgb(config.map.color.axis), 1)
        #cv2.line(self.img_org, (self.ox, 0),  (self.ox, self.height), hex_to_rgb(config.map.color.axis), 1)
        #ticks = np.arange(-self.width/2*self.csize, self.width/2*self.csize + 0.5, 1)
        #for i in ticks:
        #    if i == 0:
        #        continue
        #    tick_x = int(i / self.csize)
        #    cv2.line(self.img_org, (self.ox - tick_x, self.oy-10), (self.ox - tick_x, self.oy + 10), hex_to_rgb(config.map.color.axis), 1)
        #    cv2.line(self.img_org, (self.ox -10, self.oy - tick_x), (self.ox + 10, self.oy- tick_x), hex_to_rgb(config.map.color.axis), 1)

        self.current_x = 0
        self.current_y = 0
        self.current_a = 0
        self.path = []
    
        self.gmap = np.zeros((self.height, self.width), dtype=np.uint8)
        self.scan_px = np.zeros(config.slam.scan_data_size)
        self.scan_py = np.zeros(config.slam.scan_data_size)

    def get_path(self):
        return self.path

    def update(self, ts, urg_data, odo):
        skip_data = config.slam.skip
        data_size = config.slam.scan_data_size
        start_angle = config.lidar.start_angle
        step_angle = config.lidar.step_angle
        deg2pi = math.pi/180
        
        for index, d in enumerate(urg_data):
            r = d[1]/1000
            th = (start_angle + step_angle * index)*deg2pi
            self.scan_px[index] = r * np.cos(th)
            self.scan_py[index] = r * np.sin(th)
        if self.count == 0:
            self.count += 1
            self.gmap = update_map(self.gmap, self.scan_px, self.scan_py, self.current_x, self.current_y, self.current_a, self.width, self.height, self.ox, self.oy, self.csize)
            gmap_show(self.gmap, self.width, self.height, self.path, 10)

            self.path.append([ts, self.current_x, self.current_y, self.current_a])
            return
    
        # 現在姿勢 + odoを中心とした探索窓をnp.meshgrid()で作成する
        ccx = odo.rx
        ccy = odo.ry
        print(f"ccx:{ccx} ccy:{ccy}")
        ddx = np.arange(-0.5 + ccx, 0.5 + ccx, self.csize)
        ddy = np.arange(-0.5 + ccy, 0.5 + ccy, self.csize)
        #ddx = np.arange(-0.5 + self.current_x, 0.5 + self.current_x, self.csize)
        #ddy = np.arange(-0.5 + self.current_y, 0.5 + self.current_y, self.csize)
        max_length = 15 # 回転方向を探索する際の1ピクセルを超えないことを保証する距離
                        # 長い方が処理時間は伸びる(delta_thが細かくなるため)
        delta_th = math.acos(1 - self.csize**2/max_length**2)
        dth = np.arange(-math.pi/4 + odo.ra, math.pi/4 + odo.ra, delta_th)
        #dth = np.arange(-math.pi/4 + self.current_a + odo.dth, math.pi/4 + self.current_a + odo.dth, delta_th)
        cx, cy, ca = np.meshgrid(ddx, ddy, dth)
        cs = np.cos(ca)
        sn = np.sin(ca)
        
        eval_matrix = np.zeros_like(cx)     # 探索窓内での候補点に対する評価値
        for i in range(0, data_size, skip_data):    # スキャン点に対してループを作成する
            xd, yd = transf(self.scan_px[i], self.scan_py[i], cx, cy, cs, sn)
            ix, iy = xy2index(xd, yd, self.csize, self.ox, self.oy)
            mask = (0 <= ix) & (ix < self.width) & (0 <= iy) & (iy < self.height)
            eval_matrix[mask] += self.gmap[iy[mask], ix[mask]]
        
        best_index = np.unravel_index(np.argmax(eval_matrix), eval_matrix.shape)
        self.current_x, self.current_y, self.current_a = cx[best_index], cy[best_index], ca[best_index]
        odo.rx = self.current_x
        odo.ry = self.current_y
        odo.ra = self.current_a
        
        self.gmap = update_map(self.gmap, self.scan_px, self.scan_py, self.current_x, self.current_y, self.current_a, self.width, self.height, self.ox, self.oy, self.csize)
        self.path.append([ts, self.current_x, self.current_y, self.current_a])
        #print(f"{current_x:.3f} {current_y:.3f} {current_a*180/pi:.1f}")
        gmap_show(self.gmap, self.width, self.height, self.path, 10)
        self.count += 1
