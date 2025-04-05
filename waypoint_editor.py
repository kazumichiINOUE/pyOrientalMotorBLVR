import cv2
import modules_py.ReadConfig as rc
import numpy as np

class DrawText():
    def __init__(self, font=cv2.FONT_HERSHEY_SIMPLEX, font_scale=5, font_thickness=5, text_color=(123,123,123)):
        self.font = font
        self.font_scale = font_scale
        self.font_thickness = font_thickness
        self.text_color = text_color

    def get_text_size(self, text):
        (text_width, text_height), _ = cv2.getTextSize(text, self.font, self.font_scale, self.font_thickness)
        return text_width, text_height
    
    def draw_text(self, img, text, x, y):
        cv2.putText(img, text, (x, y), self.font, self.font_scale, self.text_color, self.font_thickness)

map_dir_path = "./experiment_250405-0"
wp_filepath = map_dir_path + "/wp_list.txt"
path_filepath = map_dir_path + "/global_map/rebuild_robot_poses.txt"
mapInfo = rc.read_mapInfo(f"{map_dir_path}/global_map/mapInfo.lua")
wp_list = []
robot_poses = []
with open(wp_filepath, 'r') as file:
    for line in file:
        parts = line.strip().split(',')
        x = float(parts[0])
        y = float(parts[1])
        wp_list.append([x, y])
with open(path_filepath, 'r') as file:
    for line in file:
        parts = line.strip().split(' ')
        x = float(parts[1])
        y = float(parts[2])
        robot_poses.append([x, y])

img = cv2.imread(f"{map_dir_path}/global_map/rebuild_opt.png", cv2.COLOR_GRAY2BGR)
img[img == 123] = 255
img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

height, width = img.shape[:2]  # 先頭2要素を取得
# 座標軸を描画する
if 1:
    cv2.line(img_color, (0, mapInfo.originY), (width, mapInfo.originY), (55, 55, 55), 1, cv2.LINE_AA)
    cv2.line(img_color, (mapInfo.originX, 0), (mapInfo.originX, height), (55, 55, 55), 1, cv2.LINE_AA)

# SLAMの軌跡
for p in robot_poses:
    x, y = p 
    ix = int(x/mapInfo.csize + mapInfo.originX)
    iy = int(-y/mapInfo.csize + mapInfo.originY)
    cv2.circle(img_color, (ix, iy), 10, (155, 155, 155), -1, cv2.LINE_AA)

wp_line_list = []
for wp in wp_list:
    x, y = wp
    ix = int(x/mapInfo.csize + mapInfo.originX)
    iy = int(-y/mapInfo.csize + mapInfo.originY)
    cv2.circle(img_color, (ix, iy), 10, (255, 0, 0), 2, cv2.LINE_AA)
    wp_line_list.append((ix, iy))

prev = wp_line_list[0]
for ind in range(1, len(wp_line_list)):
    cv2.line(img_color, prev, wp_line_list[ind], (255, 0, 0), 1, cv2.LINE_AA, )
    prev = wp_line_list[ind]

# 全体を囲む線を描画
# 0のピクセルを見つける
binary_mask = (img == 0).astype(np.uint8)
bx, by, bw, bh = cv2.boundingRect(binary_mask)
cv2.rectangle(img_color, (bx, by), (bx + bw, by + bh), (0, 0, 255), 2)  # (0, 0, 255)はBGRで赤色

# 地図領域のサイズ表示
dtext = DrawText()
text_w = str(f"{bw*mapInfo.csize:.0f}m")
text_width, text_height = dtext.get_text_size(text_w)
x = img.shape[1]//2 - text_width//2
y = by
dtext.draw_text(img_color, text_w, x, y)

text_h = str(f"{bh*mapInfo.csize:.0f}m")
x = bx + bw
y = by + bh//2
dtext.draw_text(img_color, text_h, x, y)

# Waypointに座標を表示
wtext = DrawText(font_scale=2)
for ind, wp in enumerate(wp_list):
    x, y = wp
    ix = int(x/mapInfo.csize + mapInfo.originX) + 50
    iy = int(-y/mapInfo.csize + mapInfo.originY) - 50
    text = str(f"{ind+1}({x}, {y})")
    wtext.draw_text(img_color, text, ix, iy)

# ファイル名を表示
filename_text = DrawText(font_scale=3)
text = f"{wp_filepath}"
filename_text.draw_text(img_color, text, bx, 100)

cv2.imshow("map", img_color)
cv2.imwrite("wplist.png", img_color)
cv2.waitKey()
