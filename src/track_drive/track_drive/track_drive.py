import multiprocessing
from turtle import speed
import rclpy, time, cv2, os, math
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor 
from rclpy.qos import qos_profile_sensor_data
from xycar_msgs.msg import XycarMotor
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
# from nav_msgs.msg import Odometry
from .with_yolo_traffic_light import with_yolo_traffic_light
from .child_zone_v2 import ChildZoneDetector
from .line import LineTraceNode
from .rabacon import RabaconDrive

class TrackDriverNode(Node):

    #=============================================
    # 클래스 생성 초기화 함수
    #=============================================
    def __init__(self):

        super().__init__('driver')
        self.get_logger().info('----- Xycar self-driving node started -----')
        
        # 상수값 및 초기값 설정
        self.first = True
        self.pass_cnt = 0
        self.lidar_no_scan_cnt = 0
        self.go_straight_start_time = None
        self.aspect_ratio = 0.0
        
        self.latest_cv_image = None
        self.bridge = CvBridge()
        
        self.frame_count = 0
        self.yolo_interval = 5
        self.kp = 55.0
        self.max_angle = 100.0
        self.min_dist = 0.05
        self.max_dist = 3.0
        
        self.base_speed = 0.0
        self.base_angle = 0.0
        
        self.line_module = LineTraceNode(standalone=False)
        self.child_zone_module = ChildZoneDetector(standalone=False)
        # self.rabar_module = RabaconDrive()
        
        self.main_callback_group = ReentrantCallbackGroup()
        
        self.traffic_status = "NONE" 
        self.drive_status = "NONE"
        self.car = False
        
        self.image_sub = self.create_subscription(Image, '/usb_cam/image_raw/front', self.image_callback, 1, callback_group=self.main_callback_group)
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data, callback_group=self.main_callback_group)
        self.traffic_sub = self.create_subscription(String, '/traffic_light_status', self.traffic_status_callback, 10)
        self.traffic_big_sub = self.create_subscription(String, '/traffic_big', self.traffic_big_callback, 10)
        self.car_sub = self.create_subscription(String, '/car_status', self.car_status, 10)
        # self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10) # QoS 프로파일 (일반적으로 10 사용)
        self.motor_pub = self.create_publisher(XycarMotor, '/xycar_motor', 10)
        
        self.get_logger().info("🏎️ 마스터 주행 통합 노드가 가동되었습니다. (Multi Thread 구조)")
    
    def traffic_status_callback(self, msg):
        if self.go_straight_start_time == None:
            if msg.data == "GO" and self.traffic_status != "GO":
                self.go_straight_start_time = time.time() # 현재 시간 기록
            
        self.traffic_status = msg.data
        if self.traffic_status == "GO":
            self.first = False
    
    def traffic_big_callback(self, msg):
        if msg.data == "big":
            # print("big")
            self.line_module.fast = True
            
    def car_status(self, msg):
        if msg.data == "car":
            self.car = True
        else:
            self.car = False
            
        
    def image_callback(self, msg):
        if self.drive_status == "RABACON":
            return
        
        if self.first and self.traffic_status != "GO":
            return
        
        if self.traffic_status != "NONE": 
            # print("here?")
            if self.traffic_status == "STOP":
                self.base_speed = 0.0
                self.publish_motor(0.0, 0.0)
                return
            elif self.traffic_status == "YELLOW":
                if self.base_speed != 0:
                    self.base_speed = 0.2
                # print("YELLOW")
            elif self.traffic_status == "GO":
                self.base_speed = 12.0
                # print("GO")
        
        if self.traffic_status == "GO" and self.go_straight_start_time is not None:
            if time.time() - self.go_straight_start_time < 1.5:
                # print("🚦 [교차로 통과 중] 차선 인식을 우회하고 강제 직진합니다.")
                self.publish_motor(speed=10.0, angle=0.0) 
                return 
        self.latest_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        if self.latest_cv_image is None:
            return
        
        frame = self.latest_cv_image

        self.child_zone_module.image_callback(frame)
        if self.child_zone_module.zone_status == "NORMAL":
            # print("normal so speed and angle by line module")
            self.line_module.image_callback(frame)
            self.base_speed = self.line_module.base_speed
            # print(f"speed : {self.base_speed}")
            self.base_angle = self.line_module.angle_deg
        else:
            self.line_module.standard_d = 520
            # self.base_speed = 5
            # print('child zone')
            # print('this is school zone angle: ', self.base_angle)
            
        # 디버깅용
        # self.get_logger().info(f"light detect : {self.traffic_light_module.traffic_light_detected}")
        # self.get_logger().info(f"status is {self.traffic_light_module.signal_status}")
        # print(f'speed {self.base_speed}')
        # self.publish_motor(speed=self.base_speed, angle=self.base_angle)
        
        # self.publish_motor(speed=local_speed, angle=local_angle)
        
        
    def publish_motor(self, speed, angle):
    #     """ 모터 토픽 발행을 전담하는 헬퍼 함수 """
        # print(f"💌 [MOTOR OUT] Speed Command: {speed} | Angle: {angle}")
        motor_msg = XycarMotor()
        motor_msg.speed = float(speed)
        motor_msg.angle = float(angle)
        self.motor_pub.publish(motor_msg)
        
    def normalize_angle(self, angle_deg):
        """0~360도 각도를 -180~180도로 변환"""
        while angle_deg > 180:
            angle_deg -= 360
        while angle_deg < -180:
            angle_deg += 360
        return angle_deg


    def get_range_values(self, scan, deg_min, deg_max):
        values = []

        for i, dist in enumerate(scan.ranges):
            if math.isinf(dist) or math.isnan(dist):
                continue

            if dist < self.min_dist or dist > self.max_dist:
                continue

            if 0.14 <= dist <= 0.17:
                continue

            angle = scan.angle_min + scan.angle_increment * i
            angle_deg = math.degrees(angle)
            angle_deg = self.normalize_angle(angle_deg)

            if deg_min <= angle_deg <= deg_max:
                values.append(dist)

        return values

    def mean_or_none(self, values):
        if len(values) == 0:
            return None
        return float(np.mean(values))

    def scan_callback(self, scan):
        left_values = self.get_range_values(scan, 20, 80)
        right_values = self.get_range_values(scan, -80, -20)
        front_values = self.get_range_values(scan, -15, 15)

        left_dist = self.mean_or_none(left_values)
        right_dist = self.mean_or_none(right_values)
        front_dist = self.mean_or_none(front_values)

        # print('_____________________')
        # print(left_dist, left_values)
        # print(right_dist, right_values)
        # print(front_dist, front_values)
        # print('_____________________')
     
        
        if (left_dist is not None and left_dist > 1.2 and len(left_values) >= 2) and \
           (right_dist is not None and right_dist > 1.2 and len(right_values) >= 2) and \
            front_dist is None:
            self.drive_status = "RABACON"
        # print(left_dist, left_values)
        # print('_____________________')
        # print(right_dist, right_values)
        
        if self.drive_status == "RABACON":
            if len(left_values) == 0 and len(right_values) == 0:
                self.lidar_no_scan_cnt += 1

                if self.lidar_no_scan_cnt > 3:
                    self.lidar_no_scan_cnt = 0
                    self.drive_status = "NORMAL"
                    # print("status : Rabacon -> Normal")
                return 
            else:
                self.lidar_no_scan_cnt = 0
        
        if self.drive_status == "RABACON":    
            # print("In RABACON")
            angle = 0.0
            speed = 10.0

            if left_dist is not None and right_dist is not None:
                error = left_dist - right_dist
                angle = self.kp * error -30.0

            elif left_dist is not None:
                angle = 55.0

            elif right_dist is not None:
                angle = -85.0

            else:
                angle = -20.0
                speed = 8.0

            if front_dist is not None and front_dist < 0.6:
                speed = 8.0

            angle = max(min(angle, self.max_angle), -self.max_angle)

            self.publish_motor(speed, angle)
            
        ranges = scan.ranges
        
        # 1. 시야 구역 세분화 (노이즈 구간 99~262 완벽 배제)
        # ① 정면 중앙 (-15도 ~ +15도)
        f_center = [d for d in list(ranges[345:360]) + list(ranges[0:15]) if math.isfinite(d)]
        # ② 좌측 앞 (15도 ~ 45도) : 왼쪽 차선이 비었는지 확인
        f_left = [d for d in ranges[15:45] if math.isfinite(d)]
        # ③ 우측 앞 (315도 ~ 345도) : 오른쪽 차선이 비었는지 확인
        f_right = [d for d in ranges[315:345] if math.isfinite(d)]
        
        # ④ 좌측 측면 (45도 ~ 90도) : 우측 추월 시 상대차가 왼쪽에 있는지 확인
        s_left = [d for d in ranges[45:90] if math.isfinite(d)]
        # ⑤ 우측 측면 (270도 ~ 315도) : 좌측 추월 시 상대차가 오른쪽에 있는지 확인
        s_right = [d for d in ranges[270:315] if math.isfinite(d)]

        # 각 구역별 최소 거리 계산 (데이터가 없으면 무한대로 처리)
        min_f_center = min(f_center) if f_center else float('inf')
        min_f_left = min(f_left) if f_left else float('inf')
        min_f_right = min(f_right) if f_right else float('inf')
        min_s_left = min(s_left) if s_left else float('inf')
        min_s_right = min(s_right) if s_right else float('inf')
        
        # 2. 상태 머신 (양방향 FSM)
        if self.car and self.drive_status == 'NORMAL' and self.child_zone_module.zone_status != 'SCHOOL_ZONE':
            if min_f_center < 7 and len(f_center) > 10:  # 정면에 차량 감지
                # 뚫린 차선 판단 (좌/우측 앞 공간 비교)
                if min_f_left > min_f_right:
                    self.pass_direction = 'LEFT'
                    self.get_logger().warn(f"정면 막힘! 좌측 공간 확인됨({min_f_left:.1f}m). 좌측 회피 시작!")
                else:
                    self.pass_direction = 'RIGHT'
                    self.get_logger().warn(f"정면 막힘! 우측 공간 확인됨({min_f_right:.1f}m). 우측 회피 시작!")
                
                self.drive_status = 'AVOID'

        elif self.drive_status == 'AVOID':
            if self.pass_direction == 'LEFT':
                self.get_logger().info("좌측으로 차선 변경 중...")
                self.line_module.standard_d = 600
            else:
                self.get_logger().info("우측으로 차선 변경 중...")
                self.line_module.standard_d = 440
                self.line_module.publish_motor(5, self.line_module.angle_deg)
                
            self.pass_cnt += 1

            # 차선을 완전히 넘어와서 정면이 뚫렸다면 직진(추월) 상태로 전환
            if min_f_center > 5 and self.pass_cnt > 30:
                self.pass_cnt = 0
                self.get_logger().warn("차선 진입 완료, 직진하며 추월합니다.")
                self.drive_status = 'PASSING'
                

        elif self.drive_status == 'PASSING':
            self.get_logger().info("추월 차선에서 직진 중...")
            self.line_module.publish_motor(15, self.line_module.angle_deg)
            # 내가 좌측으로 피했다면, 상대차는 내 '우측'에 있음
            if self.pass_direction == 'LEFT' and min_s_right > 5.0:
                # self.get_logger().warn("우측 상대 차량 통과 완료! 차선 복귀 시작")
                self.drive_status = 'RETURN'
            # 내가 우측으로 피했다면, 상대차는 내 '좌측'에 있음
            elif self.pass_direction == 'RIGHT' and min_s_left > 5.0:
                # self.get_logger().warn("좌측 상대 차량 통과 완료! 차선 복귀 시작")
                self.drive_status = 'RETURN'

        elif self.drive_status == 'RETURN':
            # 피했던 방향의 반대 방향으로 조향하여 복귀
            if self.pass_direction == 'LEFT':
                self.get_logger().info("우측으로 조향하여 본래 차선 복귀 중...")
                # [TODO] 우측으로 스티어링 꺾기 (예: -0.3)
                if self.pass_cnt < 10:
                    self.line_module.standard_d = 520
                    self.line_module.alpha = 0.6
                elif self.pass_cnt < 20:
                    self.line_module.standard_d = 490
                    self.line_module.alpha = 0.6
                elif self.pass_cnt < 30:
                    self.line_module.standard_d = 450
                    self.line_module.alpha = 0.6
                
            elif self.pass_direction == 'RIGHT' :
                self.get_logger().info("좌측으로 조향하여 본래 차선 복귀 중...")
                # [TODO] 좌측으로 스티어링 꺾기 (예: +0.3)
                if self.pass_cnt < 10:
                    self.line_module.standard_d = 520
                    self.line_module.alpha = 0.6
                elif self.pass_cnt < 20:
                    self.line_module.standard_d = 550
                    self.line_module.alpha = 0.6
                elif self.pass_cnt < 30:
                    self.line_module.standard_d = 590
                    self.line_module.alpha = 0.6
            self.line_module.publish_motor(18, self.line_module.angle_deg)
            self.pass_cnt += 1
            
            # 복귀 완료 조건 (정면이 충분히 뚫렸고, 다시 중앙에 자리 잡았다고 가정)
            if min_f_center > 5.0 and min_s_left > 2.0 and min_s_right > 2.0: 
                if self.pass_cnt > 30:
                    self.pass_cnt = 0
                    self.get_logger().warn("차선 복귀 완료! 일반 주행 전환")
                    self.drive_status = 'NORMAL'
                    self.pass_direction = None # 방향 초기화
                    self.line_module.alpha = 0.8
                    self.line_module.publish_motor(18, self.line_module.angle_deg)
                
    
    # def odom_callback(self, msg):
    #     curr_x = msg.pose.pose.position.x
    #     curr_y = msg.pose.pose.position.y

    #     # 처음 데이터가 들어왔을 때 초기화
    #     if self.prev_x is None or self.prev_y is None:
    #         self.prev_x = curr_x
    #         self.prev_y = curr_y
    #         return

    #     # 직전 위치와 현재 위치 사이의 이동 거리(피타고라스 정리) 계산하여 누적
    #     dx = curr_x - self.prev_x
    #     dy = curr_y - self.prev_y
    #     self.total_distance += math.sqrt(dx**2 + dy**2)

    #     # 직전 위치 업데이트
    #     self.prev_x = curr_x
    #     self.prev_y = curr_y

    #     self.odom_cnt += 1
    #     # [측정용 로그] 차를 수동으로 움직여보면서 추월 구간의 시작점과 끝점 거리를 메모하세요!
    #     print(f"현재 누적 이동 거리: {self.total_distance:.2f} m")
            

def run_traffic_light():
    rclpy.init()
    node = with_yolo_traffic_light()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
def run_driver():
    """ 🟢 독립된 OS 프로세스 A에서 돌아갈 주행 제어 루틴 """
    rclpy.init()
    node = TrackDriverNode() 
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        

        
#=============================================
# 메인 함수
#=============================================
def main(args=None):
    multiprocessing.set_start_method('spawn')
    # 1. 신호등 프로세스 생성 (서브 코어 할당)
    traffic_process = multiprocessing.Process(target=run_traffic_light)
    # 2. 주행 제어 프로세스 생성 (메인 코어 할당)
    driver_process = multiprocessing.Process(target=run_driver)
    # 3. 두 개의 OS 프로세스를 동시에 백그라운드에서 스타트!
    traffic_process.daemon = True
    driver_process.daemon = True
    traffic_process.start()
    driver_process.start()
    try:
        # 두 프로세스가 모두 끝날 때까지 메인 스레드는 대기하고 제어합니다.
        # 이 덕분에 Ctrl+C를 누르면 두 프로세스가 동시에 깔끔하게 죽습니다.
        driver_process.join()
        traffic_process.join()
    except KeyboardInterrupt:
        print("\n👋 자율주행 시스템을 안전하게 종료합니다.")
        traffic_process.terminate()
        driver_process.terminate()

if __name__ == '__main__':
    main()

