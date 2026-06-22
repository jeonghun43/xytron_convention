#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from xycar_msgs.msg import XycarMotor # 제공된 대회용 모터 메시지 타입
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class TrafficLightNode(Node):
    def __init__(self):
        super().__init__('traffic_light_node')
        
        # 1. Topic Subscriber & Publisher 설정
        # 전방 카메라 이미지 구독
        self.image_sub = self.create_subscription(
            Image,
            '/usb_cam/image_raw/front',
            self.image_callback,
            10
        )
        
        # 모터 제어 명령 발행
        self.motor_pub = self.create_publisher(XycarMotor, '/xycar_motor', 10)
        
        # ROS 이미지를 OpenCV 이미지로 변환하기 위한 Bridge
        self.bridge = CvBridge()
        
        # 신호등 상태 플래그 (한 번 출발하면 계속 직진하도록 설정)
        self.is_green_light = False
        self.is_green_light = False
        self.signal_status = "NONE"
        
        self.get_logger().info("🚦 신호등 인식 노드가 시작되었습니다. 녹색불 대기 중...")

    def image_callback(self, data):
        try:
            # ROS 이미지 메시지를 OpenCV 이미지(BGR)로 변환
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # 2. ROI (관심 영역) 설정
        # 화면 중앙 상단의 신호등 영역만 잘라냅니다. (시뮬레이터 화면 해상도에 맞게 튜닝 필요)
        # 예시: 세로 100~200, 가로 250~390 픽셀 영역
        roi = cv_image[90:200, 180:390]
        
        if roi.size == 0:
            return

        # 3. 색상 변환 (BGR -> HSV) 및 필터링
        hsv = cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
        
        # 녹색(Green) 계열의 HSV 범위 설정 (대회 시뮬레이터 조명에 따라 미세조정 필요)
        lower_green = np.array([35, 100, 100])
        upper_green = np.array([85, 255, 255])
        lower_red = np.array([0, 80, 80])
        upper_red = np.array([10, 255, 255])
        
        # 지정한 범위 내의 픽셀만 흰색(255)으로 추출하는 마스크 생성
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        red_mask = cv2.inRange(hsv, lower_red, upper_red)
        
        # 4. 픽셀 개수 카운트
        green_pixels = cv2.countNonZero(green_mask)
        red_pixels = cv2.countNonZero(red_mask)
        
        self.get_logger().info(f"🟢픽셀 개수 : {green_pixels} ")
        self.get_logger().info(f"🔴픽셀 개수 : {red_pixels} ")
        # 5. 판단 및 모터 제어
        if not self.is_green_light:
            if green_pixels > 2000: # 임계값(Threshold) 이상 녹색 픽셀이 보이면
                # self.get_logger().info(f"🟢 녹색불 감지! (픽셀 수: {green_pixels}) 출발합니다!")
                self.is_green_light = True
                self.signal_status = "GO"
                
            elif red_pixels > 1000: # 임계값 이상 빨간색 픽셀이 보이면
                # self.get_logger().info(f"🔴 빨간불 감지! (픽셀 수: {red_pixels}) 정지합니다!")
                # self.publish_motor(speed=0.0, angle=0.0) # 정지 명령 발행
                self.signal_status = "STOP"
            else:
                # self.publish_motor(speed=0.0, angle=0.0)
                # self.signal_status = "YELLOW"
                pass
        
        # 녹색불이 켜진 이후에는 계속 앞으로 전진
        if self.is_green_light:
            self.publish_motor(speed=5.0, angle=0.0) # 속도 30으로 직진 (최대치 확인 필요)

        # 디버깅을 위한 화면 출력 (제출 전에는 주석 처리 또는 제거하는 것이 속도에 좋습니다)
        cv2.imshow("ROI", roi)
        cv2.imshow("Green Mask", green_mask)
        cv2.imshow("Red Mask", red_mask)
        cv2.waitKey(1)

    def publish_motor(self, speed, angle):
        """ 모터 제어 메시지 발행 함수 """
        motor_msg = XycarMotor()
        motor_msg.speed = float(speed)
        motor_msg.angle = float(angle)
        self.motor_pub.publish(motor_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 노드가 종료되었습니다.")
    finally:
        node.publish_motor(speed=0.0, angle=0.0) # 종료 시 차량 정지
        node.destroy_node()
        rclpy.shutdown()
        # cv2.destroyAllWindows()

if __name__ == '__main__':
    main()