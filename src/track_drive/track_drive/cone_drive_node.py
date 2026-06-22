#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import XycarMotor
import numpy as np

class ConeDriveNode(Node):
    """
    라이다(LiDAR) 센서 데이터를 받아 좌우 라바콘 벽면 사이의 거리를 계산하고,
    차량이 중앙을 유지하며 정밀하게 곡선 코스를 주행하도록 제어하는 ROS2 노드입니다.
    """
    def __init__(self):
        super().__init__('cone_drive_node')
        
        # [구독자 및 발행자 초기화]
        # 시뮬레이터의 라이다 데이터(/scan) 구독 및 모터 제어 토픽(/xycar_motor) 발행 설정
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.motor_pub = self.create_publisher(XycarMotor, '/xycar_motor', 10)
        
        # [제어 파라미터 설정]
        # 주행 속도 및 조향 비례 게인(Kp) 설정 (시뮬레이터 반응을 보며 최적화 필요)
        self.drive_speed = 1.0  # 라바콘 구간 안정화를 위한 저속 주행 설정
        self.kp = 30.0           # 좌우 오차에 따른 조향각 가중치 (비례 게인)
        
        self.get_logger().info("🔶 라바콘 인식 주행 노드가 시작되었습니다. 라이다 데이터를 분석합니다.")

    def scan_callback(self, msg):
        """
        라이다 센서 프레임이 들어올 때마다 호출되는 콜백 함수입니다.
        전방 좌측과 우측 섹터를 나누어 라바콘과의 거리를 연산한 뒤 조향 명령을 계산합니다.
        """
        left_ranges = []
        right_ranges = []
        
        # [전방 센서 데이터 필터링 및 섹터 분류]
        # 표준 ROS 라이다 규격(정면 0도, 좌측 +, 우측 - 라디안)을 기준으로 
        # 전방 좌측(15°~60°)과 전방 우측(-60°~-15°) 영역의 라바콘 거리를 분리하여 수집합니다.
        for i, distance in enumerate(msg.ranges):
            # 유효하지 않은 데이터(inf, nan, 범위를 벗어난 측정값)는 제외합니다.
            if distance < msg.range_min or distance > msg.range_max or np.isnan(distance) or np.isinf(distance):
                continue
                
            # 배열 인덱스를 라디안 각도로 변환
            angle = msg.angle_min + (i * msg.angle_increment)
            angle = np.arctan2(np.sin(angle), np.cos(angle))
            
            # 전방 좌측 섹터 필터링 (약 15도 ~ 60도 범위)
            if 0.26 <= angle <= 1.05:
                left_ranges.append(distance)
            # 전방 우측 섹터 필터링 (약 -60도 ~ -15도 범위)
            elif -1.05 <= angle <= -0.26:
                right_ranges.append(distance)

        # [좌우 대표 거리 산출]
        # 각 섹터에 감지된 라바콘 포인트들의 평균 거리를 구해 벽면까지의 대표 거리로 사용합니다.
        # 감지된 점이 없다면 안전을 위해 기본 최대 거리(3.0m)로 치환합니다.
        left_dist = np.mean(left_ranges) if len(left_ranges) > 0 else 3.0
        right_dist = np.mean(right_ranges) if len(right_ranges) > 0 else 3.0

        # [조향 오차 계산 및 비례 제어(P-Control)]
        # 차량이 중앙에서 벗어난 오차(Error)를 계산합니다.
        # 우측 공간이 더 넓으면(right_dist > left_dist) 에러가 양수가 되어 우회전 조향(+ angle)을 유도하고,
        # 좌측 공간이 더 넓으면 음수가 되어 좌회전 조향(- angle)을 수행하도록 기하학적 메커니즘을 구성합니다.
        error = right_dist - left_dist
        steering_angle = error * self.kp
        
        # 하드웨어 및 시뮬레이터 최대 조향 제한 범위로 데이터 클리핑 (예: -50 ~ 50도 제한)
        steering_angle = max(-100.0, min(100.0, steering_angle))
        
        # 2. 라이다 센서가 바라본 좌/우측 라바콘 점의 개수와 평균 거리를 출력합니다.
        self.get_logger().info(
            f"[인지] 좌측 🟡 점 {len(left_ranges):3d}개 ({left_dist:.2f}m) "
            f"↔ 우측 🔵 점 {len(right_ranges):3d}개 ({right_dist:.2f}m)"
        )

        # 3. 계산된 오차값과 그에 따라 최종 선택한 조향 방향/속도를 출력합니다.
        self.get_logger().info(
            f"[결정] 거리오차(L-R): {error:+.2f}m ➔ 선택 행동: {steering_angle} | 속도: {self.drive_speed:.1f}\n"
        )

        # [모터 제어 토픽 발행]
        # 연산된 속도와 조향각을 XycarMotor 메시지에 담아 시뮬레이터 차량으로 전송합니다.
        self.publish_motor(self.drive_speed, steering_angle)

    def publish_motor(self, speed, angle):
        """ 제어 명령 전송을 전담하는 헬퍼 함수입니다. """
        motor_msg = XycarMotor()
        motor_msg.header.stamp = self.get_clock().now().to_msg()
        motor_msg.speed = float(speed)
        motor_msg.angle = float(angle)
        self.motor_pub.publish(motor_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ConeDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 라바콘 주행 노드가 종료되었습니다.")
    finally:
        node.publish_motor(speed=0.0, angle=0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()