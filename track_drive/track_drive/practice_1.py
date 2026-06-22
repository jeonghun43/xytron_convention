#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#=============================================
# 버드아이뷰(IPM) 기반 자이카 라인 추종 알고리즘 (슬라이딩 메모리 추적 버전)
#=============================================
import rclpy, time, cv2, os, math
import numpy as np
from rclpy.node import Node
from xycar_msgs.msg import XycarMotor
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge

class BirdEyeHyperNode(Node):

    def __init__(self):
        super().__init__('bird_eye_hyper_driver')
        self.get_logger().info('----- Bird-Eye Hyper Drive Node Started -----')
        
        self.image = None  
        self.motor_msg = XycarMotor()        
        self.bridge = CvBridge()
        
        self.curr_steering_angle = 0.0
        
        # ★ 차선 위치를 추적하기 위한 메모리 변수 초기화
        self.prev_left_x = None
        self.prev_right_x = None
        
        self.motor_pub = self.create_publisher(XycarMotor, 'xycar_motor', 10)
        self.sub_front = self.create_subscription(
            Image, '/usb_cam/image_raw/front', self.cam_callback, qos_profile_sensor_data)

        # 버드아이뷰 원근 변환 매트릭스
        self.src_pts = np.float32([
            [120, 320],   # 좌상
            [520, 320],   # 우상
            [620, 440],   # 우하
            [20, 440]     # 좌하
        ])
        
        self.dst_pts = np.float32([
            [150, 0],
            [490, 0],
            [490, 480],
            [150, 480]
        ])

    def cam_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
   
    def drive(self, angle, speed):
        self.motor_msg.angle = float(angle)
        self.motor_msg.speed = float(speed)
        self.motor_pub.publish(self.motor_msg)

    def get_bird_eye_view(self, frame):
        height, width = frame.shape[:2]
        M = cv2.getPerspectiveTransform(self.src_pts, self.dst_pts)
        warped = cv2.warpPerspective(frame, M, (width, height))
        return warped

    # =========================================================================
    # [★핵심 수정★] 위치 추적 기반 차선 분류 알고리즘
    # =========================================================================
    def process_lane_pixels(self, warped_mask):
        height, width = warped_mask.shape
        scan_y = int(height * 0.60) 
        scan_line = warped_mask[scan_y, :]
        
        pixel_indices = np.where(scan_line > 0)[0]
        if len(pixel_indices) > 0:
            white_ratio = len(pixel_indices) / width
            if white_ratio > 0.20: # 스캔라인의 35% 이상이 흰색이면 체크무늬 고속도로 상황!
                # 조향을 틀지 않도록 무조건 유실 처리하여 직전 조향을 유지하게 만듭니다.
                return None, None, scan_y, True
        if len(pixel_indices) == 0:
            return None, None, scan_y, False
            
        # 초기 구동 시(직전 기억이 없을 때) 기본 경계값 설정
        if self.prev_left_x is None: self.prev_left_x = int(width * 0.25)
        if self.prev_right_x is None: self.prev_right_x = int(width * 0.75)
        
        # 주행 속도가 빠르므로 커브 시 차선이 급격히 이동할 수 있음을 감안한 추적 윈도우 마진
        window_margin = int(width * 0.35) 
        
        # 직전 위치 기준으로 윈도우 안의 픽셀만 추출
        left_pixels = pixel_indices[(pixel_indices >= self.prev_left_x - window_margin) & 
                                    (pixel_indices <= self.prev_left_x + window_margin)]
                                    
        right_pixels = pixel_indices[(pixel_indices >= self.prev_right_x - window_margin) & 
                                     (pixel_indices <= self.prev_right_x + window_margin)]
        
        # 픽셀 평균값 계산 및 메모리 업데이트
        if len(left_pixels) > 0:
            left_x = np.mean(left_pixels)
            self.prev_left_x = left_x
        else:
            left_x = None
            # 유실되었을 때는 윈도우가 가만히 굳어있지 않고 화면 왼쪽 바깥으로 서서히 밀어둠
            self.prev_left_x = max(0, self.prev_left_x - 10) 
            
        if len(right_pixels) > 0:
            right_x = np.mean(right_pixels)
            self.prev_right_x = right_x
        else:
            right_x = None
            self.prev_right_x = min(width, self.prev_right_x + 10)
            
        # 두 차선이 너무 가까워져 하나로 뭉치는 오류 방지 (동일 픽셀 오인 사격 차단)
        if left_x is not None and right_x is not None:
            if abs(left_x - right_x) < int(width * 0.15):
                left_x = None # 좌회전 중 우측 차선이 침범한 것이 유력하므로 좌측을 날림
                
        return left_x, right_x, scan_y, False

    def main_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.image is None: continue
            
            frame = self.image.copy()
            bev_frame = self.get_bird_eye_view(frame)
            
            hsv = cv2.cvtColor(bev_frame, cv2.COLOR_BGR2HSV)
            # 노란색 점선은 원천 차단
            mask = cv2.inRange(hsv, np.array([0, 0, 180]), np.array([180, 50, 255]))
            
            left_x, right_x, scan_y, is_checkerboard = self.process_lane_pixels(mask)
            
            mid = frame.shape[1] // 2
            target_x = mid
            
            # =========================================================================
            # 제어 로직 (차선 오인이 해결되어 사진 속 상황에서 정확히 right_x로만 들어옵니다)
            # =========================================================================
            if is_checkerboard:
                target_angle=self.curr_steering_angle
            elif left_x is not None and right_x is not None:
                target_x = int((left_x + right_x) / 2)
                x_offset = target_x - mid
                y_offset = int(frame.shape[0] * 0.25) 
                target_angle = int(math.atan2(x_offset, y_offset) * 180.0 / math.pi)
            
            
            elif right_x is not None:
                # ★[보내주신 사진 상태] 정중앙을 넘어와도 우측 차선으로 완벽 추적 -> 즉시 좌측 풀 락!!★
                target_angle = -100.0
                target_x = int(right_x - 165)
                
            elif left_x is not None:
                # 우회전 탈선 위기 -> 우측 풀 락
                target_angle = 100.0
                target_x = int(left_x + 165)
                
            else:
                target_angle = self.curr_steering_angle

            self.curr_steering_angle = self.curr_steering_angle * 0.7 + target_angle * 0.3
            final_angle = max(-100.0, min(100.0, self.curr_steering_angle))
            
            # 초고속 하이퍼 스케줄러 속도 제어
            if abs(final_angle) >= 48.0:
                target_speed = 4.0  # 확실하게 감속하여 풀 스티어링 상태에서 슬립으로 궤도 복귀
            elif abs(final_angle) >= 20.0:
                target_speed = 7.0  
            else:
                target_speed = 10.0  
                
            self.drive(angle=final_angle, speed=target_speed)
            
            # 디버깅 비주얼라이저
            debug_bev = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            cv2.line(debug_bev, (mid, 0), (mid, frame.shape[0]), (0, 0, 255), 2) 
            if is_checkerboard:
                cv2.putText(debug_bev, "CHECKERBOARD DETECTED! HOLD", (10, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            if left_x is not None:
                cv2.circle(debug_bev, (int(left_x), scan_y), 8, (255, 0, 0), -1) 
            if right_x is not None:
                # 확인해 보세요! 이제 저 하얀 선이 빨간 선 왼쪽에 있어도 초록색 점(Right)이 찍힙니다!
                cv2.circle(debug_bev, (int(right_x), scan_y), 8, (0, 255, 0), -1) 
            if (left_x is not None or right_x is not None) and not is_checkerboard:
                cv2.circle(debug_bev, (target_x, scan_y), 12, (0, 255, 255), -1) 
                
            cv2.imshow("Bird-Eye View Mask", debug_bev)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

def main(args=None):
    rclpy.init(args=args)
    node = BirdEyeHyperNode()
    try: node.main_loop()
    except KeyboardInterrupt: pass
    finally:
        node.drive(angle=0, speed=0)
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()