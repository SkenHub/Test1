import sys           # システム関連のモジュール。プログラムの終了などに使用。
import pygame        # ゲーム開発用ライブラリpygameをインポート

def main():
    # pygameライブラリの初期化。内部モジュールの設定や使用するリソースの準備を行う
    pygame.init()
    
    # ジョイスティック（ゲームパッド）を扱うためのサブモジュールを初期化
    pygame.joystick.init()

    # 接続されているジョイスティックの数を取得する
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        # ジョイスティックが接続されていない場合は警告メッセージを出力後、プログラムを終了する
        print("ジョイスティックが接続されていません。")
        sys.exit(1)  # 異常終了を示す終了コード1でプログラム終了

    # 0番目のジョイスティックを取得し、初期化する
    # "/dev/input/js0"が0番目のジョイスティックに対応している前提
    joystick = pygame.joystick.Joystick(0)
    joystick.init()  # ジョイスティック固有の初期化処理を実施
    # 初期化したジョイスティックの名前を表示して、どのデバイスが使用されているか確認
    print("初期化されたジョイスティック:", joystick.get_name())

    try:
        while True:
            # pygame.event.get()でイベントキューからすべてのイベントを取得し、各イベントに対して処理を行う
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    # 軸（アナログスティックなど）の動きのイベント
                    # 各軸の値が -1.0～1.0 の範囲で表現されるため、小数点3桁で出力
                    print(f"軸 {event.axis} の値: {event.value:.3f}")
                elif event.type == pygame.JOYBUTTONDOWN:
                    # ボタンが押された時のイベント。どのボタンの入力かを表示
                    print(f"ボタン {event.button} が押されました。")
                elif event.type == pygame.JOYBUTTONUP:
                    # ボタンが離された時のイベント
                    print(f"ボタン {event.button} が離されました。")
                elif event.type == pygame.JOYHATMOTION:
                    # ハットスイッチ（方向パッド）の動きのイベント
                    print(f"ハット {event.hat} の値: {event.value}")
    except KeyboardInterrupt:
        # Ctrl+Cなどのキーボード割り込みによりプログラムが中断された場合の処理
        print("終了します...")
    finally:
        # プログラム終了時にpygameで確保したリソースを適切に解放する処理
        pygame.quit()

if __name__ == "__main__":
    # このスクリプトが直接実行された場合にmain()関数を呼び出す
    main()