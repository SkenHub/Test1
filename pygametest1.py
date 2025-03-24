import sys
import pygame

def main():
    pygame.init()
    pygame.joystick.init()

    # 接続されているジョイスティックの数を取得
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("ジョイスティックが接続されていません。")
        sys.exit(1)

    # 0番目のジョイスティックを初期化（"/dev/input/js0"がこれに対応する前提）
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("初期化されたジョイスティック:", joystick.get_name())

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    print(f"軸 {event.axis} の値: {event.value:.3f}")
                elif event.type == pygame.JOYBUTTONDOWN:
                    print(f"ボタン {event.button} が押されました。")
                elif event.type == pygame.JOYBUTTONUP:
                    print(f"ボタン {event.button} が離されました。")
                elif event.type == pygame.JOYHATMOTION:
                    print(f"ハット {event.hat} の値: {event.value}")
    except KeyboardInterrupt:
        print("終了します...")
    finally:
        pygame.quit()

if __name__ == "__main__":
    main()