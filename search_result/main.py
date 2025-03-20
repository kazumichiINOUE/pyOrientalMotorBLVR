import numpy as np
import matplotlib.pyplot as plt

counter = 0
ave_x = []
ave_y = []
ave_a = []
var_x = []
var_y = []
var_a = []
eval_list = []
trajectory = []
cov_matrix_list = [] 

while True:
    fname = f"search_result_{counter}.txt"
    try:
        with open(fname, "r") as file:
            poses = []
            for ind, line in enumerate(file):
                vals = line.split()  # スペース区切りでリストにする
                x, y, a, eval = vals
                poses.append([float(x), float(y), float(a), int(eval)])
            # NumPy の配列に変換
            poses = np.array(poses)  # shape: (N, 4)
            # x, y, a のみ抽出
            xyz = poses[:, :3]  # shape: (N, 3)
            # 平均
            mean = np.mean(xyz, axis=0)
            # 分散
            var = np.var(xyz, axis=0)
            var_x.append(var[0])
            var_y.append(var[1])
            var_a.append(var[2])
            # 共分散行列 (3×3)
            cov_matrix = np.cov(xyz, rowvar=False)
            cov_matrix_list.append(cov_matrix)
            # 結果表示
            #print(f"counter: {counter} num_of_data: {ind}")
            #print(f"平均: {mean}")  # [xの平均, yの平均, aの平均]
            #print(f"分散: {var}")  # [xの分散, yの分散, aの分散]
            #print(f"共分散行列:\n{cov_matrix}")
            #print()
            eval_list.append(poses[-1][3])
            trajectory.append(poses[-1])
    except FileNotFoundError:
        print(f"{fname} が見つからないため終了")
        break  # ファイルがない場合ループを抜ける

    counter += 1  # 次のファイルへ

# x 軸 (インデックス)
indices = np.arange(len(var_x))

# グラフ作成
fig, ax1 = plt.subplots()

# 左軸 (Y1) に var_x, var_y, var_a をプロット
ax1.set_xlabel("Index")
ax1.set_ylabel("Variance (x, y, a)", color="tab:blue")
ax1.plot(indices, var_x, marker="", linestyle="-", color="red", label="Var x")
ax1.plot(indices, var_y, marker="", linestyle="-", color="green", label="Var y")
ax1.plot(indices, var_a, marker="", linestyle="-", color="blue", label="Var a")
ax1.tick_params(axis="y", labelcolor="tab:blue")
ax1.legend(loc="upper left")

# 右軸 (Y2) に eval_list をプロット
ax2 = ax1.twinx()
ax2.set_ylabel("Eval", color="tab:orange")
ax2.plot(indices, eval_list, marker="", linestyle="-", color="orange", label="Eval")
ax2.tick_params(axis="y", labelcolor="tab:orange")
ax2.legend(loc="upper right")

# 表示
plt.title("Variance and Eval Graph")
plt.show()

fig, ax = plt.subplots(figsize=(10,10))
trajectory = np.array(trajectory)
ax.plot(trajectory[:,0], trajectory[:,1])
ax.set_xlabel("X[m]")
ax.set_ylabel("Y[m]")
ax.grid(True)


from matplotlib.patches import Ellipse
for ind, cov in enumerate(cov_matrix_list):
    if ind % 10 == 0:
        cov_matrix_2x2 = cov[:2, :2]
        x_mean, y_mean, _, _ = trajectory[ind]
        eigvals, eigvecs = np.linalg.eig(cov_matrix_2x2)
        width, height = 2 * np.sqrt(eigvals)  # 半径ではなく直径にするため2倍
        angle = np.degrees(np.arctan2(eigvecs[1, 0], eigvecs[0, 0]))
        ellipse = Ellipse(xy=(x_mean, y_mean), width=width, height=height, 
                  angle=angle, edgecolor='r', facecolor='gray', alpha=0.3, linewidth=0)
        ax.add_patch(ellipse)
        ax.plot(x_mean, y_mean, 'bo', label='Center')
        offsets = [(0.1, 0.1), (-0.1, -0.1)]  # 表示位置の調整
        for i in range(2):
            text_x = x_mean + offsets[i][0]
            text_y = y_mean + offsets[i][1]
            ax.text(text_x, text_y, f'λ{i+1}={eigvals[i]:.2f}', fontsize=5h, color='black')
    else:
        continue
ax.set_aspect('equal') 
plt.show()