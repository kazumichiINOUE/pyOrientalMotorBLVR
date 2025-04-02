import pygame
import sys

# Pygameを初期化
pygame.init()

# ジョイスティックを初期化
pygame.joystick.init()

# 接続されているジョイスティックの数を確認
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("ジョイスティックが接続されていません")
    sys.exit()

# 最初のジョイスティックを取得
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"検出されたジョイスティック: {joystick.get_name()}")

# メインループ
try:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            
            # ボタンの押下を検出
            if event.type == pygame.JOYBUTTONDOWN:
                print(f"ボタン {event.button} が押されました")
            
            # ボタンの解放を検出
            if event.type == pygame.JOYBUTTONUP:
                print(f"ボタン {event.button} が離されました")
            
            # 軸の動きを検出
            if event.type == pygame.JOYAXISMOTION:
                print(f"軸 {event.axis} の値: {joystick.get_axis(event.axis):.2f}")
            
            # ハットスイッチの動きを検出
            if event.type == pygame.JOYHATMOTION:
                print(f"ハット {event.hat} の値: {joystick.get_hat(event.hat)}")

        pygame.time.wait(10)  # 短い待機時間を入れてCPU使用率を下げる

except KeyboardInterrupt:
    print("\nプログラムを終了します")
    pygame.quit()
    sys.exit()
