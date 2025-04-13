import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty  # サービス用
import math

class RoboWareNode(Node):
    def __init__(self):
        super().__init__('roboware_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'send_can_message', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'robot_position',
            self.robot_position_callback,
            10
        )
        # 複数の目標座標と目標角度を設定 (X, Y, θ)
        self.target_positions = [
            [-2050.0, 0.0, 0.0],#L
            [-2050.0, 1212.0, 0.0],#kago_get
            [-1100.0, 1212.0, 0.0],#tonneru_in
            [-1100.0, 2287.0, 0.0],#F
            [-350.0, 2287.0, 0.0],#L
            [-350.0, 3324.0, 0.0],#F
            [-1100.0, 3324.0, 0.0],#R
            [-1100.0, 4361.0, 0.0],#F
            [-350.0, 4361.0, 0.0],#L
            [-350.0, 5443.0, 0.0],#tonneru_out
            [-1750.0, 5443.0, 0.0],#R
            [-1750.0, 6462.0, 0.0],#F
            [-462.0, 6462.0, 0.0],#ball_get
            [-1750.0, 6462.0, 0.0],#R
            [-1750.0, 5443.0, 0.0],#B
            [-2962.0, 5443.0, 0.0],#ball_shoot
            [-350.0, 5443.0, 0.0],#tonneru_in
            [-350.0, 4361.0, 0.0],#B
            [-1100.0, 4361.0, 0.0],#corn_get
            [-1100.0, 3324.0, 0.0],#B
            [-350.0, 3324.0, 0.0],#L
            [-350.0, 2287.0, 0.0],#B
            [-1100.0, 2287.0, 0.0],#R
            [-1100.0, 1212.0, 0.0],#tonneru_out
            [-2050.0, 1212.0, 0.0],#gado_put
        ]
        self.current_target_index = 0  # 現在の目標座標のインデックス
        self.current_position = [0.0, 0.0, 0.0]  # 現在の座標 (X, Y, θ)
        self.action_number = 0  # 指示番号
        self.is_active = False  # 動作開始フラグ (初期状態は停止)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 100msごとに実行

        # PID制御用のパラメータ
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.0
        self.kp_theta = 1.0  # 角度制御用の比例ゲイン
        self.previous_error = [0.0, 0.0]
        self.integral = [0.0, 0.0]

        # サービスを作成して動作開始を制御
        self.start_service = self.create_service(
            Empty, 'start_robot', self.start_robot_callback
        )

    def start_robot_callback(self, request, response):
        self.is_active = True  # 動作を開始
        self.get_logger().info("ロボットの動作を開始します")
        return response

    def robot_position_callback(self, msg):
        # マイコンから送られてきた現在のロボット位置を更新
        self.current_position = msg.data

    def timer_callback(self):
        if not self.is_active:
            return  # 動作開始フラグがFalseの場合は何もしない

        if self.current_target_index >= len(self.target_positions):
            self.get_logger().info("全ての目標座標に到達しました")
            self.is_active = False  # 動作を停止
            return

        # 現在の目標座標と目標角度を取得
        target_position = self.target_positions[self.current_target_index]

        # PID制御で速度を計算
        error_x = target_position[0] - self.current_position[0]
        error_y = target_position[1] - self.current_position[1]

        self.integral[0] += error_x
        self.integral[1] += error_y

        derivative_x = error_x - self.previous_error[0]
        derivative_y = error_y - self.previous_error[1]

        vx = self.kp * error_x + self.ki * self.integral[0] + self.kd * derivative_x
        vy = self.kp * error_y + self.ki * self.integral[1] + self.kd * derivative_y

        self.previous_error = [error_x, error_y]

        # 目標方向の角度を計算
        target_theta = target_position[2]
        current_theta = self.current_position[2]
        error_theta = target_theta - current_theta

        # 角度誤差を -π ～ π の範囲に正規化
        error_theta = (error_theta + math.pi) % (2 * math.pi) - math.pi

        # 角速度を計算
        omega = self.kp_theta * error_theta

        # 目標座標と目標角度に到達したら次の目標に移行
        if math.sqrt(error_x**2 + error_y**2) < 0.1 and abs(error_theta) < 0.1:  # 許容誤差
            self.get_logger().info(f"目標座標 {target_position} に到達しました")
            self.current_target_index += 1  # 次の目標座標に移行
            self.action_number += 1  # 指示番号を変更
            if self.current_target_index < len(self.target_positions):
                self.get_logger().info(f"次の目標座標: {self.target_positions[self.current_target_index]}")

        # CANメッセージを送信
        data = Float32MultiArray()
        data.data = [vx, vy, omega, float(self.action_number)]
        self.publisher_.publish(data)

def main(args=None):
    rclpy.init(args=args)
    node = RoboWareNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()