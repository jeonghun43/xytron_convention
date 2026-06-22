#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import XycarMotor
import numpy as np

class OvertakeController(Node):
    """
    라이다(/scan) 데이터를 바탕으로 앞차의 상대 좌표를 실시간 계산하여 동적 목표점을 설정하고,
    퓨어 퍼수트(Pure Pursuit) 기하학 공식을 이용해 추월 조향각을 연산하는 자วน주행 노드입니다.
    """
    def __init__(self):
        super().__init__('overtake_controller')
        
        # [1. 통신 인터페이스 설정]
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.motor_pub = self.create_publisher(XycarMotor, '/xycar_motor', 10)
        
        # [2. 주행 파라미터 및 퓨어 퍼수트 설정]
        self.overtake_state = "NORMAL"
        self.base_speed = 10.0
        self.pass_speed = 15.0
        self.action_counter = 0
        
        # 자이카(Xycar) 하드웨어 규격 기준 축거 (Wheelbase: 앞바퀴와 뒷바퀴 축 사이의 거리, 약 30cm)
        self.wheelbase = 0.3 
        
        self.get_logger().info("🏎️ 퓨어 퍼수트 기반 동적 추월 제어 노드가 가동되었습니다.")

    def scan_callback(self, msg):
        if self.action_counter > 0:
            self.action_counter -= 1

        # 각도 정규화 및 섹터별 거리 데이터 분리
        front_distances = []       # 진입 판단용 정면 영역 (-15도 ~ +15도)
        left_side_distances = []   # 안전 확보용 좌측 사각지대 (45도 ~ 90도)
        
        # 🚨 [동적 추적 핵심] 추월 주행 중 앞차를 지속해서 쫓아가기 위한 확장 추적 영역
        # 차가 왼쪽으로 비껴가면 앞차는 내 차량 기준 '우측 후방'으로 이동하므로 전방과 우측 영역을 넓게 감시합니다.
        track_distances = []
        track_angles = []
        
        for i, distance in enumerate(msg.ranges):
            if distance < msg.range_min or distance > msg.range_max or np.isnan(distance) or np.isinf(distance):
                continue
                
            angle = msg.angle_min + (i * msg.angle_increment)
            angle = np.arctan2(np.sin(angle), np.cos(angle)) # -pi ~ +pi 정규화
            
            # ① 일반 정면 섹터 필터링
            if -0.26 <= angle <= 0.26:
                front_distances.append(distance)
            # ② 좌측 사각지대 섹터 필터링
            elif 0.78 <= angle <= 1.57:
                left_side_distances.append(distance)

            # ③ 추월 상태 주행 시 앞차의 위치를 실시간 추적하기 위한 섹터 (-60도 ~ +15도)
            if -1.05 <= angle <= 0.26:
                track_distances.append(distance)
                track_angles.append(angle)

        # 각 섹터별 대표 최소 거리 산출
        min_front_dist = np.min(front_distances) if len(front_distances) > 0 else 5.0
        min_left_dist = np.min(left_side_distances) if len(left_side_distances) > 0 else 5.0

        # ----------------------------------------------------------------------
        # [3. 추월 의사결정 FSM 및 동적 조향각 연산 부모 블록]
        # ----------------------------------------------------------------------
        if self.overtake_state == "NORMAL":
            # 앞차가 1.5m 이내로 들어오고 좌측 차선이 비어있으면 추월 모드 진입
            if min_front_dist < 1.5:
                if min_left_dist > 2.0:
                    self.get_logger().warn("🚨 앞차 감지! 동적 추월 주행을 기동합니다.")
                    self.overtake_state = "OVERTAKE_PHASE_1"
                else:
                    self.get_logger().error("🛑 왼쪽 차선 차단! 충돌 방지를 위해 긴급 정지합니다.")
                    self.publish_motor(speed=0.0, angle=0.0)
                    return
            else:
                # 일반 상황: 차선 유지 기본 전진 (0.0도)
                self.publish_motor(speed=self.base_speed, angle=0.0)

        elif self.overtake_state == "OVERTAKE_PHASE_1":
            # [4. 동적 추월 경로 생성 및 퓨어 퍼수트 추종 블록]
            if len(track_distances) > 0:
                # 추적 범위 내에서 가장 가까운 앞차의 거리와 각도 포착
                min_idx = np.argmin(track_distances)
                obs_dist = track_distances[min_idx]
                obs_angle = track_angles[min_idx]
                
                # 극좌표계(거리, 각도) 데이터를 차량 기준의 직교 좌표계(X: 전방, Y: 좌측)로 변환
                obs_x = obs_dist * np.cos(obs_angle)
                obs_y = obs_dist * np.sin(obs_angle)
                
                # 💡 만약 앞차의 전방 위치(obs_x)가 내 차 뒤로 넘어갔다면(-0.2m 이하) 추월이 끝난 것입니다.
                if obs_x < -0.2:
                    self.get_logger().info("🟢 앞차 추월 완료. 복귀 모드로 전환합니다.")
                    self.overtake_state = "OVERTAKE_PHASE_2"
                    self.action_counter = 45 # 복귀 조향을 유지할 안정 타이머 작동
                    return

                # [하드코딩 타파] 앞차 위치 기준 왼쪽 대각선 앞으로 '동적 목표점(Waypoint)' 계산
                # 앞차보다 1.2m 앞(X축), 그리고 왼쪽 차선인 왼쪽으로 0.7m(Y축) 떨어진 지점을 실시간 타겟 지정
                target_x = obs_x + 1.2
                target_y = obs_y + 0.7
                
                # [퓨어 퍼수트 기하학 수식 적용]
                # Steering = atan2(2 * L * sin(alpha), Ld) 공식을 간소화한 차량 제어 표준 공식
                Ld_squared = target_x**2 + target_y**2
                steering_rad = (2.0 * self.wheelbase * target_y) / Ld_squared
                steering_deg = np.degrees(steering_rad)
                
                # 시뮬레이터 조향 규격 반영 (네 코드의 주석 기준: 음수(-)가 좌회전이므로 부호를 반전시킵니다)
                final_angle = -steering_deg
                final_angle = max(-50.0, min(50.0, final_angle)) # 하드웨어 보호 가드 조향 제한
                
                self.publish_motor(speed=self.pass_speed, angle=final_angle)
            else:
                # 순간적으로 앞차를 놓친 예외 상황 시 안전을 위해 복귀 모드로 스위칭
                self.overtake_state = "OVERTAKE_PHASE_2"
                self.action_counter = 45

        elif self.overtake_state == "OVERTAKE_PHASE_2":
            # [5. 가상 목표점을 이용한 원래 차선 안전 복귀 블록]
            if self.action_counter > 0:
                # 오른쪽 차선으로 복귀하기 위해 차량 우측 전방 방향으로 '가상의 복귀 목표점'을 강제 세팅
                # 전방 2.0m 앞, 우측(Y-) 방향으로 -0.6m 지점을 조향 타겟으로 설정
                virtual_target_x = 2.0
                virtual_target_y = -0.6
                
                # 복귀 경로 퓨어 퍼수트 계산
                Ld_squared = virtual_target_x**2 + virtual_target_y**2
                steering_rad = (2.0 * self.wheelbase * virtual_target_y) / Ld_squared
                steering_deg = np.degrees(steering_rad)
                
                # 부호 규격 반영 (virtual_target_y가 음수이므로 계산 결과는 자동으로 양수(+) 우회전이 됩니다)
                final_angle = -steering_deg
                final_angle = max(-50.0, min(50.0, final_angle))
                
                self.publish_motor(speed=self.base_speed, angle=final_angle)
            else:
                # 복귀 타이머가 종료되면 주행 모드를 다시 정상 차선 추종으로 완전 초기화
                self.get_logger().info("🏁 복귀 완료. 일반 차선 주행 모드로 환원합니다.\n")
                self.overtake_state = "NORMAL"

    def publish_motor(self, speed, angle):
        motor_msg = XycarMotor()
        motor_msg.speed = float(speed)
        motor_msg.angle = float(angle)
        self.motor_pub.publish(motor_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OvertakeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# from xycar_msgs.msg import XycarMotor
# import numpy as np

# class OvertakeController(Node):
#     """
#     라이다(/scan) 데이터를 분석하여 전방의 저속 방해 차량을 감지하고,
#     측방 사각지대의 안전을 확보한 뒤 추월 경로 조향 명령을 발행하는 ROS2 노드입니다.
#     """
#     def __init__(self):
#         super().__init__('overtake_controller')
        
#         # [1. 통신 인터페이스 설정]
#         self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
#         self.motor_pub = self.create_publisher(XycarMotor, '/xycar_motor', 10)
        
#         # [2. 추월 FSM 상태 변수 초기화]
#         # "NORMAL": 일반 차선 추종, "OVERTAKE_PHASE_1": 왼쪽으로 차선 변경, "OVERTAKE_PHASE_2": 오른쪽으로 복귀
#         self.overtake_state = "NORMAL"
#         self.base_speed = 10.0
#         self.pass_speed = 15.0
#         # 차선 변경 조향 유지를 위한 시간 타이머 카운터
#         self.action_counter = 0
        
#         self.get_logger().info("🏎️ 방해 차량 추월 주행 제어 노드가 가동되었습니다.")

#     def scan_callback(self, msg):
#         if self.action_counter > 0:
#             self.action_counter -= 1

#         # 각도 정규화(지난 번 라바콘에서 배운 -pi ~ +pi 변환 적용)
#         front_distances = [] # 정면 영역 (-15도 ~ +15도)
#         left_side_distances = [] # 좌측 사각지대 (약 45도 ~ 90도)
        
#         for i, distance in enumerate(msg.ranges):
#             if distance < msg.range_min or distance > msg.range_max or np.isnan(distance) or np.isinf(distance):
#                 continue
                
#             angle = msg.angle_min + (i * msg.angle_increment)
#             angle = np.arctan2(np.sin(angle), np.cos(angle)) # 각도 정규화
            
#             # [3. 차량 정면 섹터 필터링 (-15도 ~ 15도)]
#             if -0.26 <= angle <= 0.26:
#                 front_distances.append(distance)
#             # [4. 좌측 측방 사각지대 섹터 필터링 (45도 ~ 90도)]
#             elif 0.78 <= angle <= 1.57:
#                 left_side_distances.append(distance)

#         # 각 섹터별 최소 거리 계산 (가장 가까운 장애물 타겟 기준)
#         min_front_dist = np.min(front_distances) if len(front_distances) > 0 else 5.0
#         min_left_dist = np.min(left_side_distances) if len(left_side_distances) > 0 else 5.0

#         # [5. 추월 의사결정 FSM 부모 블록]
#         if self.overtake_state == "NORMAL":
#             # 앞차와의 거리가 1.5미터 이내로 극도로 가까워졌을 때 추월 시동!
#             if min_front_dist < 1.5:
#                 # 좌측 사각지대 2.0미터 이내에 아무도 없다면 추월 감행
#                 if min_left_dist > 2.0:
#                     self.get_logger().warn("🚨 [TRAFFIC ALERT] 전방 차량 감지! 왼쪽으로 추월 차선 변경을 시작합니다.")
#                     self.overtake_state = "OVERTAKE_PHASE_1"
#                     self.action_counter = 45  # 약 1.5초 동안 강제 조향 유지 인터벌 설정
#                 else:
#                     # 왼쪽 차선이 막혀있다면 안전을 위해 급정거 (후진하는 차 대비 가드)
#                     self.get_logger().error("🛑 [DANGER] 앞차는 가까우나 왼쪽 차선이 막힘! 긴급 정지합니다.")
#                     self.publish_motor(speed=0.0, angle=0.0)
#                     return
#             else:
#                 # 일반 주행 상황 (원래는 팀원의 차선유지 조향각(lane_angle)을 넣어줘야 합니다)
#                 self.publish_motor(speed=self.base_speed, angle=0.0)

#         elif self.overtake_state == "OVERTAKE_PHASE_1":
#             # [6. 왼쪽 차선 변경 제어 분기]
#             # 타이머 카운터가 도는 동안 강제로 바퀴를 왼쪽(-)으로 꺾어 옆 차선으로 넘어갑니다.
#             # (시뮬레이터 조향 규격: 음수(-)가 좌회전)
#             if self.action_counter > 0:
#                 self.publish_motor(speed=self.pass_speed, angle=-50.0)
#             else:
#                 # 차선 변경이 완료되면 다음 단계(앞차를 지나칠 때까지 직진)로 스위칭
#                 self.get_logger().info("🟢 추월 차선 진입 완료. 앞차를 추월 중입니다.")
#                 self.overtake_state = "OVERTAKE_PHASE_2"
#                 self.action_counter = 60 # 약 2초간 추월 직진 주행 시간 부여

#         elif self.overtake_state == "OVERTAKE_PHASE_2":
#             # [7. 앞차 추월 후 원래 차선 복귀 제어 분기]
#             if self.action_counter > 0:
#                 # 앞차를 추월하며 시원하게 직진 주행
#                 self.publish_motor(speed=self.base_speed, angle=0.0)
#             else:
#                 # 2초간 직진하여 앞차를 완전히 제쳤으므로, 다시 오른쪽(-)으로 핸들을 꺾어 원래 차선 복귀
#                 self.get_logger().info("🏁 추월 완료! 원래 주행 차선으로 복귀합니다.")
#                 self.publish_motor(speed=self.base_speed, angle=-35.0)
                
#                 # 완전히 복귀할 시간(약 1.5초)을 준 뒤 상태를 NORMAL로 완전 초기화
#                 self.overtake_state = "NORMAL"
#                 self.action_counter = 45

#     def publish_motor(self, speed, angle):
#         motor_msg = XycarMotor()
#         motor_msg.speed = float(speed)
#         motor_msg.angle = float(angle)
#         self.motor_pub.publish(motor_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = OvertakeController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()