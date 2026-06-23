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
from .with_yolo_traffic_light import with_yolo_traffic_light
from .child_zone import SchoolZoneDetector
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
        self.image = None  # 카메라 토픽 데이터를 저장할 변수
        self.lidar_ranges = None
        self.lidar_no_scan_cnt = 0
        self.go_straight_start_time = None
        
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
        
        self.image_sub = self.create_subscription(Image, '/usb_cam/image_raw/front', self.image_callback, 10, callback_group=self.main_callback_group)
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data, callback_group=self.main_callback_group)
        self.traffic_sub = self.create_subscription(String, '/traffic_light_status', self.traffic_status_callback, 10)
        self.motor_pub = self.create_publisher(XycarMotor, '/xycar_motor', 10)
        
        self.get_logger().info("🏎️ 마스터 주행 통합 노드가 가동되었습니다. (Multi Thread 구조)")
    
    def traffic_status_callback(self, msg):
        if self.go_straight_start_time == None:
            if msg.data == "GO" and self.traffic_status != "GO":
                self.go_straight_start_time = time.time() # 현재 시간 기록
            
        self.traffic_status = msg.data
        if self.traffic_status == "GO":
            self.first = False
        
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
                print("🚦 [교차로 통과 중] 차선 인식을 우회하고 강제 직진합니다.")
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
            # self.base_speed = 5
            print('child zone')
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
                    print("status : Rabacon -> Normal")
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
      
    # 1. 신호등 프로세스 생성 (서브 코어 할당)
    traffic_process = multiprocessing.Process(target=run_traffic_light)
    # 2. 주행 제어 프로세스 생성 (메인 코어 할당)
    driver_process = multiprocessing.Process(target=run_driver)
    # 3. 두 개의 OS 프로세스를 동시에 백그라운드에서 스타트!
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

