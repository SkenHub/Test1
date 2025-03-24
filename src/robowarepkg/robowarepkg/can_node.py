# このファイルは、CANバスを介してメッセージの送受信を実装するためのモジュールです。
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import can
import struct
import time

class CANNode(Node):
    def __init__(self):
        # コンストラクタ: ノード初期化およびCANバス設定
        super().__init__('can_node')

        # CANバスの設定
        try:
            self.bus = can.Bus(interface='socketcan', channel='can0')
        except OSError as e:
            self.get_logger().error(f"Could not access SocketCAN device can0: {e}")
            rclpy.shutdown()
            return

        self.publisher_ = self.create_publisher(Float32MultiArray, 'robot_position', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'send_can_message',
            self.send_can_message_callback,
            10
        )
        self.timer = self.create_timer(0.001, self.timer_callback)  # 1msに一回
        self.action_number = 0
        self.last_msg_time = self.get_clock().now()

    def send_can_message_callback(self, msg):
        # 受信したFloat32MultiArrayメッセージからCANメッセージを構築し送信する
        self.get_logger().info(f"send_can_message_callback called with data: {msg.data}")
        self.last_msg_time = self.get_clock().now()
        if len(msg.data) == 4:
            vx = float(msg.data[0])  # Vx
            vy = float(msg.data[1])  # Vy
            omega = float(msg.data[2])  # ω
            action_number = int(msg.data[3])  # 指示番号

            # action_numberを0から10の範囲に制限
            action_number = max(0, min(10,action_number))

            # CANデータのパッキング
            data_160 = struct.pack('>hhh', int(vx), int(vy), int(omega))
            can_msg_160 = can.Message(arbitration_id=0x160, data=data_160, is_extended_id=False)
            try:
                self.bus.send(can_msg_160)  # CANバスに送信
                self.get_logger().info(f"送信[0x160: {data_160.hex()}] 送信[{vx}, {vy}, {omega}]")
            except can.CanError:
                self.get_logger().error("Failed to send CAN message 0x160")

            data_161 = struct.pack('>B', action_number)
            can_msg_161 = can.Message(arbitration_id=0x161, data=data_161, is_extended_id=False)
            try:
                self.bus.send(can_msg_161)  # CANバスに送信
                self.get_logger().info(f"送信[0x161: {data_161.hex()}] 送信[Action Number: {action_number}]")
            except can.CanError:
                self.get_logger().error("Failed to send CAN message 0x161")

    def timer_callback(self):
        # タイマーによる定期実行: CANメッセージの受信と処理を行う
        msg = self.bus.recv(timeout=0.001)  # タイムアウトを短く設定
        if msg is not None:
            if msg.arbitration_id == 0x150:
                self.get_logger().debug(f"Received CAN message: {msg}")
                try:
                    # CANデータのアンパッキング
                    x = struct.unpack('>h', msg.data[0:2])[0]
                    y = struct.unpack('>h', msg.data[2:4])[0]
                    theta = struct.unpack('>h', msg.data[4:6])[0]
                    command_msg = Float32MultiArray()
                    command_msg.data = [float(x), float(y), float(theta), float(self.action_number)]
                    self.publisher_.publish(command_msg)
                   #self.get_logger().info(f"受信[0x150: {msg.data.hex()}] 受信[{x}, {y}, {theta}, {self.action_number}]")
                except struct.error as e:
                    self.get_logger().error(f"Unpacking error: {e}")
            elif msg.arbitration_id == 0x151:
                self.get_logger().debug(f"Received CAN message: {msg}")
                try:
                    self.action_number = struct.unpack('>B', msg.data)[0]
                   #self.get_logger().info(f"受信[0x151: {msg.data.hex()}] 受信[Action Number: {self.action_number}]")
                except struct.error as e:
                    self.get_logger().error(f"Unpacking error: {e}")
            elif msg.arbitration_id == 0x360:
                self.get_logger().debug(f"Received CAN message: {msg}")
                try:
                    # CANデータのアンパッキング
                    x = struct.unpack('>h', msg.data[0:2])[0]
                    y = struct.unpack('>h', msg.data[2:4])[0]
                    theta = struct.unpack('>h', msg.data[4:6])[0]
                    command_msg = Float32MultiArray()
                    command_msg.data = [float(x), float(y), float(theta), float(self.action_number)]
                    self.publisher_.publish(command_msg)
                   #self.get_logger().info(f"受信[0x360: {msg.data.hex()}] 受信[{x}, {y}, {theta}, {self.action_number}]")
                except struct.error as e:
                    self.get_logger().error(f"Unpacking error: {e}")

def main(args=None):
    # エントリーポイント
    rclpy.init(args=args)
    node = CANNode()
    if node.bus:
        rclpy.spin(node)
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()