import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from xycar_msgs.msg import XycarMotor
from ultralytics import YOLO
from std_msgs.msg import String

import cv2
import numpy as np


class with_yolo_traffic_light:
    def __init__(self, standalone=True):
        # super().__init__('yolo_detector')
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()
        if standalone:
            self.sub = self.create_subscription(Image, '/usb_cam/image_raw/front', self.image_callback, 10)
            self.motor_pub = self.create_publisher(XycarMotor, '/xycar_motor', 10)
        self.standalone = standalone
        
        self.status_pub = self.create_publisher(String, '/traffic_light_status', 10)
        
        # self.get_logger().info("✅ YOLOv8 ROS2 Detector Started")
        
        self.traffic_light_detected = False
        self.signal_status = False
        
    def image_callback(self, msg):
        if self.standalone:
            try:
                display_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                return
        else:
            display_frame = msg[:250, :]
            
        prediction = self.model(display_frame, conf=0.3, verbose=False)
        
        self.traffic_light_detected = False
        self.signal_status = "No_light"  # 신호등을 발견했을 때의 기본 상태는 No detect
        
        boxes = prediction[0].boxes
        for box in boxes:
            # 검출된 객체가 신호등(Class ID: 9)인 경우 처리 블록 진입
            if int(box.cls[0]) == 9:
                self.traffic_light_detected = True
                
                # 바운딩 박스 정수 좌표 계산
                xmin, ymin, xmax, ymax = map(int, box.xyxy[0])
                
                # 이미지 경계 조건 예외 처리 가드 가동
                xmin, ymin = max(0, xmin), max(0, ymin)
                xmax, ymax = min(display_frame.shape[1], xmax), min(display_frame.shape[0], ymax)
                
                aspect_ratio = (xmax - xmin) / (ymax - ymin)
                crop_ratio = int((xmax - xmin) / 4)
                # print(aspect_ratio)
                # [신호등 영역 크롭(Crop) 및 이진화 분석 부모 블록]
                roi = display_frame[ymin:ymax, xmin:xmax]
                if roi.size == 0:
                    continue
                
                #디버깅용
                # cv2.imshow("ROI", roi)
                # cv2.waitKey(1)
                
                lower_green = np.array([35, 80, 80])
                upper_green = np.array([85, 255, 255])
                lower_red1 = np.array([0, 50, 50])
                upper_red1 = np.array([10, 255, 255])
                lower_red2 = np.array([170, 50, 50])
                upper_red2 = np.array([180, 255, 255])
                lower_yellow = np.array([11, 80, 80])
                upper_yellow = np.array([34, 255, 255])
                
                hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

                red_mask1 = cv2.inRange(hsv_roi, lower_red1, upper_red1)
                red_mask2 = cv2.inRange(hsv_roi, lower_red2, upper_red2)
                red_mask = cv2.bitwise_or(red_mask1, red_mask2)  # 🌟 양쪽 범위의 빨간색 합치기
                yellow_mask = cv2.inRange(hsv_roi, lower_yellow, upper_yellow)
                straight_mask = cv2.inRange(hsv_roi, lower_green, upper_green)
                
                red_pixels = cv2.countNonZero(red_mask)
                yellow_pixels = cv2.countNonZero(yellow_mask)
                straight_pixels = cv2.countNonZero(straight_mask)
                
                left_pixels = 0
                
                left_roi = hsv_roi[:, 2*crop_ratio:3*crop_ratio]
                green_roi = hsv_roi[:, 3*crop_ratio:4*crop_ratio]

                if aspect_ratio >= 2.9:

                    left_mask = cv2.inRange(left_roi, lower_green, upper_green)
                    straight_mask = cv2.inRange(green_roi, lower_green, upper_green)
                    # 디버깅용
                    # cv2.imshow("red", red_mask)
                    # cv2.imshow("straight", straight_mask)
                    # cv2.imshow("left_mask", left_mask)
                    # cv2.waitKey(1)
                    left_pixels = cv2.countNonZero(left_mask)
                    straight_pixels = cv2.countNonZero(straight_mask)
                    # self.get_logger().info(f"🔴 : {red_pixels}, 🟡 : {yellow_pixels}, 🟢: {straight_pixels}")
                # 빨간색 픽셀 900개 이상이면 빨간불 = stop, 아니면 초록불 = go
                # self.get_logger().info(f"🔴 : {red_pixels}, 🟡 : {yellow_pixels}, 🟢: {straight_pixels}")
                if red_pixels >= 2000:
                    self.signal_status = "STOP"
                elif straight_pixels >= 2000:
                    self.signal_status = "GO"
                elif yellow_pixels >= 2000:
                    self.signal_status = 'YELLOW'
                    
                msg_out = String()
                if not self.traffic_light_detected:
                    msg_out.data = "NONE"
                else:
                    msg_out.data = self.signal_status # "STOP", "GO", "YELLOW" 중 하나
                
                if self.standalone:
                    pass
                else:
                    # 3. 마스터 노드를 향해 무전 쏘기!
                    self.status_pub.publish(msg_out)
                    print(f'traffic status : {msg_out.data}')
                # if aspect_ratio >= 3:
                #     cv2.imshow( "red", roi[:, :crop_ratio])
                #     cv2.imshow( "yellow", roi[:, crop_ratio:2*crop_ratio])
                #     cv2.imshow( "left_green", roi[:, 2*crop_ratio:3*crop_ratio])
                #     cv2.imshow( "straight_green", roi[:, 3*crop_ratio:4*crop_ratio])
                #     cv2.waitKey(1)
                    

    #     # [최종 모터 제어 결정 분기 블록]
    #     if traffic_light_detected:
    #         if signal_status == "GO":
    #             self.get_logger().info("🟢 초록불 감지 -> 차량 주행")
    #             self.publish_motor(speed=10, angle=0.0)
    #         elif signal_status == 'STOP':
    #             self.get_logger().info("🔴 빨간불 감지 -> 차량 정지")
    #             self.publish_motor(speed=0.0, angle=0.0)
    #         else:
    #             self.get_logger().info("🟡 노란불 감지")
    #             self.publish_motor(speed=3, angle=0.0)

    #         # 주행 중 신호등이 화면에서 사라졌다면 미션을 패스한 것이므로 기본 주행 유지
            
    # def publish_motor(self, speed, angle):
    #     """ 모터 토픽 발행을 전담하는 헬퍼 함수 """
    #     motor_msg = XycarMotor()
    #     motor_msg.speed = float(speed)
    #     motor_msg.angle = float(angle)
    #     self.motor_pub.publish(motor_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = with_yolo_traffic_light()
    rclpy.spin(node)
    node.destory_node()
    rclpy.shutdown()
    
if __name__ == main:
    main()