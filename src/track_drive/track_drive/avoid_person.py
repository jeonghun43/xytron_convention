#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

import rclpy, cv2, time, math
import numpy as np
import torch
from rclpy.node import Node
from xycar_msgs.msg import XycarMotor
from sensor_msgs.msg import Image, LaserScan
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge

class AdvancedTrackDriver(Node):
    def __init__(self):
        super().__init__('advanced_track_driver')
        self.get_logger().info('----- Advanced Auto-Driving Node Started -----')
        
        self.bridge = CvBridge()
        self.image = None  
        self.lidar_ranges = None
        
        # ROS2 통신 설정
        self.motor_pub = self.create_publisher(XycarMotor, '/xycar_motor', 10)
        self.create_subscription(Image, '/usb_cam/image_raw/front', self.cam_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile_sensor_data)
        
        # YOLOv5 커스텀 모델 로드 (대회용으로 직접 라벨링하여 학습한 가중치 사용)
        # 클래스 예시: 0:사람, 1:라바콘, 2:빨간불, 3:초록불, 4:차량, 5:경찰차
        self.get_logger().info("YOLOv5 모델 로딩 중...")
        # self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='best.pt')
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True) # 테스트용 기본 모델
        
    def cam_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
    def lidar_callback(self, msg):
        self.lidar_ranges = msg.ranges   
      
    def drive(self, angle, speed):
        msg = XycarMotor()
        msg.angle = float(max(-100, min(100, angle)))
        msg.speed = float(max(-100, min(100, speed)))
        self.motor_pub.publish(msg)

    # ==========================================
    # 1. 차선 인식 로직 (OpenCV)
    # ==========================================
    def process_lane_keeping(self):
        """ ROI 설정, 이진화, 슬라이딩 윈도우 또는 무게중심을 이용한 차선 추적 """
        img = self.image.copy()
        h, w = img.shape[:2]
        
        # ROI (관심 영역) 설정: 화면 하단 절반만 사용
        roi = img[h//2:h, 0:w]
        
        # HSV 변환 및 흰색/노란색 차선 추출 (대회장 조명에 맞춰 튜닝 필수)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 50, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)
        
        # 무게중심 계산 (단순화된 예시)
        M = cv2.moments(mask)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            # 화면 중심과 차선 중심의 차이를 각도로 변환
            error = cx - (w // 2)
            angle = error * 0.4 # P제어 게인
            return angle
        return 0.0

    def main_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.image is None or self.lidar_ranges is None:
                continue

            # 전방 거리 계산 (라이다 기준 0도 전후)
            front_ranges = list(self.lidar_ranges[0:15]) + list(self.lidar_ranges[345:360])
            valid_ranges = [r for r in front_ranges if 0.0 < r < 100.0]
            front_dist = min(valid_ranges) if valid_ranges else 100.0

            # YOLOv5 객체 탐지
            results = self.model(self.image)
            df = results.pandas().xyxy[0]
            
            # 탐지된 객체 리스트 분류
            # (실제 대회 모델 클래스 이름에 맞춰 'name' 필드 수정 필요)
            red_lights = df[df['name'] == 'red_light']
            green_lights = df[df['name'] == 'green_light']
            persons = df[df['name'] == 'person']
            cones = df[df['name'] == 'cone'] # 혹은 라바콘 클래스
            cars = df[df['name'] == 'car']
            polices = df[df['name'] == 'police']

            # 기본 주행 세팅
            base_angle = self.process_lane_keeping()
            final_angle = base_angle
            final_speed = 0

            # ==========================================
            # 판단 로직 (우선순위 제어)
            # ==========================================
            
            # [우선순위 1] 신호등 (빨간불 정지)
            if not red_lights.empty:
                self.get_logger().info("신호등: 빨간불! 정지선 대기")
                self.drive(0, 0)
                continue
            
            # [우선순위 2] 무단횡단 사람 (긴급 제동)
            if not persons.empty and front_dist < 3.0:
                self.get_logger().warning("보행자 감지! 긴급 제동!")
                self.drive(0, 0)
                continue

            # [우선순위 3] 라바콘 회피 (장애물 회피)
            if not cones.empty:
                # 가장 가까운 라바콘의 중심 X좌표 확인
                closest_cone = cones.loc[cones['ymax'].idxmax()]
                cone_x = (closest_cone['xmin'] + closest_cone['xmax']) / 2
                img_width = self.image.shape[1]
                
                # 라바콘이 왼쪽에 있으면 우회전, 오른쪽에 있으면 좌회전 가중치 부여
                if cone_x < img_width / 2:
                    final_angle += 40 # 우측으로 회피
                else:
                    final_angle -= 40 # 좌측으로 회피

            # [우선순위 4] 차량 추월
            if not cars.empty and front_dist < 4.0:
                self.get_logger().info("전방 느린 차량 발견! 추월 시도")
                # 차선을 임시로 변경하기 위해 강한 조향
                # (라이다 왼쪽/오른쪽 여유 공간을 계산하여 차선 변경 방향 결정하는 것이 이상적)
                final_angle -= 60 # 일단 좌측으로 차선 변경하여 추월한다고 가정

            # [우선순위 5] 경찰차 길목 차단 (좌회전 금지)
            if not polices.empty:
                self.get_logger().info("경찰차 길목 차단! 좌회전 금지 모드")
                # 각도가 음수(좌회전)로 계산되었을 경우, 이를 0(직진) 또는 우회전으로 강제 변환
                if final_angle < -10:
                    final_angle = 10 # 억지로 우측이나 직진으로 유도

            # ==========================================
            # 속도 제어 로직 (직선 구간 가속)
            # ==========================================
            # 조향 각도가 작을수록(직선) 속도를 올리고, 각도가 크면(커브) 감속
            if abs(final_angle) < 15:
                final_speed = 70 # 직선 최고 속도
            elif abs(final_angle) < 40:
                final_speed = 40 # 완만한 커브
            else:
                final_speed = 25 # 급커브 (안정성 우선)

            # 모터 제어 명령 전송
            self.drive(final_angle, final_speed)

            # 디버깅 화면 출력
            cv_img = np.squeeze(results.render())
            cv2.imshow("Autonomous Driving System", cv_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedTrackDriver()
    try:
        node.main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.drive(0, 0)
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()