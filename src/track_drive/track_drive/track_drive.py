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
        self.image = None  # 카메라 토픽 데이터를 저장할 변수
        self.lidar_ranges = None
        
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
        self.child_zone_module = SchoolZoneDetector(standalone=False)
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
        self.traffic_status = msg.data
        
    def image_callback(self, msg):
        if self.drive_status == "lidar":
            return
        self.latest_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        if self.latest_cv_image is None:
            return
        
        frame = self.latest_cv_image

        if self.traffic_status != "NONE":  # 신호등이 감지된 상황이라면 (detected == True 효과)
            if self.traffic_status == "STOP":
                self.base_speed = 0.0
                return
            elif self.traffic_status == "YELLOW":
                self.base_speed = 0.2
                # print("YELLOW")
            elif self.traffic_status == "GO":
                self.base_speed = 12.0
                # print("GO")

        self.child_zone_module.image_callback(frame)
        if self.child_zone_module.current_drive_mode == "NORMAL":
            # print("normal so speed and angle by line module")
            self.line_module.image_callback(frame)
            self.base_speed = self.line_module.base_speed
            # print(f"speed : {self.base_speed}")
            self.base_angle = self.line_module.angle_deg
        else:
            self.base_speed = self.child_zone_module.school_zone_speed
            print('child zone')
            # print('this is school zone angle: ', self.base_angle)
            
        # 디버깅용
        # self.get_logger().info(f"light detect : {self.traffic_light_module.traffic_light_detected}")
        # self.get_logger().info(f"status is {self.traffic_light_module.signal_status}")
        
        self.publish_motor(speed=self.base_speed, angle=self.base_angle)
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
        print("Im in LIDAR")
        self.drive_status = "lidar"
        left_values = self.get_range_values(scan, 20, 80)
        right_values = self.get_range_values(scan, -80, -20)
        front_values = self.get_range_values(scan, -15, 15)

        left_dist = self.mean_or_none(left_values)
        right_dist = self.mean_or_none(right_values)
        front_dist = self.mean_or_none(front_values)

        angle = 0.0
        speed = self.base_speed

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
        node.publish_motor(speed=0.0, angle=0.0)
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


# #!/usr/bin/env python3
# # -*- coding: utf-8 -*- 1
# #=============================================
# # 본 프로그램은 자이트론에서 제작한 것입니다.
# # 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# # 교육과 실습 용도로만 사용가능하며 외부유출은 금지됩니다.
# #=============================================
# import rclpy, time, cv2, os, math
# import numpy as np
# from rclpy.node import Node
# from xycar_msgs.msg import XycarMotor
# from sensor_msgs.msg import Image
# from sensor_msgs.msg import LaserScan
# from rclpy.qos import qos_profile_sensor_data
# from rclpy.duration import Duration
# from cv_bridge import CvBridge

# #=============================================
# # ROS2 Node 클래스 정의
# #=============================================
# class TrackDriverNode(Node):

#     #=============================================
#     # 클래스 생성 초기화 함수
#     #=============================================
#     def __init__(self):

#         super().__init__('driver')
#         self.get_logger().info('----- Xycar self-driving node started -----')
        
#         # 상수값 및 초기값 설정
#         self.image = None  # 카메라 토픽 데이터를 저장할 변수
#         self.motor_msg = XycarMotor()  # 모터토픽 메시지        
#         self.lidar_ranges = None
#         self.bridge = CvBridge()
    
#         # ROS2 Publisher & Subscriber 설정
#         self.motor_pub = self.create_publisher(XycarMotor,'xycar_motor',10)
        
#         self.sub_front = self.create_subscription(
#             Image, '/usb_cam/image_raw/front', self.cam_callback, qos_profile_sensor_data)

#         self.subscription = self.create_subscription(
#             LaserScan, '/scan', self.lidar_callback, qos_profile_sensor_data)
		
#         self.get_logger().info("Track Driver Node Initialized")
              
#     #=============================================
#     # 카메라 토픽을 수신하는 콜백 함수
#     #=============================================
#     def cam_callback(self, data):
#         # 수신한 메시지를 OpenCV 이미지로 변환하여 저장
#         self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    
#     #=============================================
#     # 라이다 토픽을 수신하는 콜백 함수
#     #=============================================
#     def lidar_callback(self, msg):
#         self.lidar_ranges = msg.ranges   
      
#     #=============================================
#     # 모터제어 토픽을 발행하는 Publisher 함수
#     #=============================================
#     def drive(self, angle, speed):
#         self.motor_msg.angle = float(angle)
#         self.motor_msg.speed = float(speed)
#         self.motor_pub.publish(self.motor_msg)

#     #=============================================
#     # 메인 루프
#     #=============================================
#     def main_loop(self):
    
#         self.get_logger().info("======================================")
#         self.get_logger().info("  S T A R T    D R I V I N G ...      ")
#         self.get_logger().info("======================================")

#         while rclpy.ok():
        
#             for _ in range(15):
#                 self.drive(angle=0,speed=0)
#                 time.sleep(0.1)

#             for _ in range(15):
#                 self.drive(angle=0,speed=5)
#                 time.sleep(0.1)
                
# #=============================================
# # 메인 함수
# #=============================================
# def main(args=None):
      
#     rclpy.init(args=args)
#     node = TrackDriverNode()
	
#     try:
#         # main_loop() 함수를 호출하여 실행합니다.
#         node.main_loop()
#     except KeyboardInterrupt:
#         # 사용자 인터럽트 (Ctrl+C)가 발생하면 예외를 처리합니다.
#         pass
#     finally:
#         # 노드를 종료하고 ROS2를 정리합니다.
#         node.drive(angle=0, speed=0)
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

