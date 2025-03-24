# このファイルは、PS3コントローラからの入力を受け取りCANメッセージとして送信するためのモジュールです。
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import pygame
import os

class PS3ControllerNode(Node):
    def __init__(self):
        # コンストラクタ: ノード初期化、ジョイスティック設定等
        super().__init__('ps3_controller_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'send_can_message', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'robot_position',
            self.robot_position_callback,
            10
        )
        
        os.environ["SDL_JOYSTICK_DEVICE"] = "/dev/input/js0"
        pygame.init()
        pygame.joystick.init()
        self.screen = pygame.display.set_mode((400, 300), pygame.RESIZABLE)
        pygame.display.set_caption("PS3 Controller Debug")
        self.font = pygame.font.Font(None, 36)
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.get_logger().info(f"Joystick initialized: {self.joystick.get_name()}")
        else:
            self.get_logger().error("No joystick found")
        self.action_number = 0  # 初期値を設定
        self.button_states = [False, False]  # 丸ボタンと四角ボタンの状態を保持
        self.robot_position = [0.0, 0.0, 0.0, 0.0]  # 受信した自己位置データ
        self.timer = self.create_timer(0.001, self.timer_callback)  # 1msに一回

    def robot_position_callback(self, msg):
        # ロボットの位置情報受信時の処理
        self.robot_position = msg.data

    def timer_callback(self):
        # 定期実行: ジョイスティック入力を取得し、CANメッセージを送信する
        pygame.event.pump()
        vx = self.joystick.get_axis(0) * 100.0  # 左ジョイスティックのx軸
        vy = self.joystick.get_axis(1) * 100.0 * -1  # 左ジョイスティックのy軸
        omega = self.joystick.get_axis(3) * 100.0  # 右ジョイスティックのx軸

        # 丸ボタン: Action Numberを増加
        if self.joystick.get_button(1):
            if not self.button_states[0]:
                self.button_states[0] = True
        else:
            if self.button_states[0]:
                if self.action_number < 10:
                    self.action_number += 1
                self.button_states[0] = False

        # 四角ボタン: Action Numberを減少
        if self.joystick.get_button(3):
            if not self.button_states[1]:
                self.button_states[1] = True
        else:
            if self.button_states[1]:
                if self.action_number > 0:
                    self.action_number -= 1
                self.button_states[1] = False

        # CANメッセージ送信用データ作成
        data = Float32MultiArray()
        data.data = [float(vx), float(vy), float(omega), float(self.action_number)]
        self.publisher_.publish(data)

        # デバッグ画面の更新
        self.screen.fill((0, 0, 0))
        text_vx = self.font.render(f"Vx: {vx:.2f}", True, (255, 255, 255))
        text_vy = self.font.render(f"Vy: {vy:.2f}", True, (255, 255, 255))
        text_omega = self.font.render(f"Omega: {omega:.2f}", True, (255, 255, 255))
        text_action_number = self.font.render(f"Action Number: {self.action_number}", True, (255, 255, 255))
        text_robot_position = self.font.render(f"Robot Position: {self.robot_position}", True, (255, 255, 255))

        # テキストの位置を自動調整
        texts = [text_vx, text_vy, text_omega, text_action_number, text_robot_position]
        y_offset = 20
        for text in texts:
            self.screen.blit(text, (20, y_offset))
            y_offset += text.get_height() + 20

        pygame.display.flip()

def main(args=None):
    # エントリーポイント
    rclpy.init(args=args)
    node = PS3ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()