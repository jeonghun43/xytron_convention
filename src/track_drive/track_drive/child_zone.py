#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from xycar_msgs.msg import XycarMotor
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import numpy as np

# class SchoolZoneDetector:
#     """
#     카메라 노면 바닥 영역(ROI)에서 대각선으로 누운 중앙 점선까지 완벽하게 필터링하여
#     '어린이보호구역' 황색 글자 덩어리만 인지하는 고도화된 노드입니다.
#     """
#     def __init__(self, standalone=True):
#         # super().__init__('school_zone_detector')
        
#         if standalone:
#             self.sub = self.create_subscription(Image, '/usb_cam/image_raw/front', self.image_callback, 10)
#             self.motor_pub = self.create_publisher(XycarMotor, '/xycar_motor', 10)
#         else:
#             self.sub = None
#             self.motor_pub = None
#         self.bridge = CvBridge()
        
#         # [어린이 보호구역 제어 플래그 변수]
#         self.current_drive_mode = "NORMAL"
#         self.school_zone_speed = 5.0  
#         self.school_zone_angle = 0.0  

#         # 쿨다운 타임 버퍼 카운터 (중복 인식 방지)
#         self.cooldown_counter = 0 
        
#         self.standalone = standalone
        
#         # self.get_logger().info("🛡️ [밀도 필터 가드 장착] 어린이 보호구역 최종 진화형 노드가 가동되었습니다.")

#     def image_callback(self, msg):
#         if self.standalone:
#             try:
#                 cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#             except CvBridgeError as e:
#                 return
#         else:
#             cv_image = msg

#     #     if self.cooldown_counter > 0:
#     #         self.cooldown_counter -= 1

#     #     # [1. 노면 바닥 영역 고정 ROI 지정]
#     #     roi = cv_image[300:400, 100:500]
        
#     #     # [2. 황색(Yellow) HSV 필터링 블록]
#     #     hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
#     #     lower_yellow = np.array([22, 180, 200])
#     #     upper_yellow = np.array([32, 255, 255])
#     #     yellow_mask = cv2.inRange(hsv_roi, lower_yellow, upper_yellow)
        
#     #     # [3. 외곽선 검출(Contour) 실행]
#     #     contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
#     #     pure_text_pixel_count = 0
#     #     debug_roi = cv2.cvtColor(yellow_mask, cv2.COLOR_GRAY2BGR)

#     #     for cnt in contours:
#     #         # 덩어리의 실제 픽셀 면적 구하기
#     #         area = cv2.contourArea(cnt)
#     #         if area < 50: # 자잘한 노이즈 선제거
#     #             continue

#     #         # 덩어리를 감싸는 최소 사각형 좌표 추출
#     #         x, y, w, h = cv2.boundingRect(cnt)
#     #         if w == 0 or h == 0:
#     #             continue
                
#     #         # 밀도(Solidity) 계산
#     #         solidity = float(area) / (w * h)
#     #    # ----------------------------------------------------------------------
#     #         # 🛡️ [수정 및 고도화된 스쿨존 가드 시스템]
#     #         # ----------------------------------------------------------------------
            
#     #         # [가드 A] 세로로 길게 관통하는 뚱뚱한 차선 검거 (기존 유지)
#     #         if h > 80:
#     #             continue
                
#     #         # [가드 B 수정] 억까 방지형 가로 정지선/횡단보도 블록 검거
#     #         # ROI 가로폭이 400(100:500)이므로, 가로 폭(w)이 200px을 넘어가면서 
#     #         # 동시에 내부가 꽉 찬(solidity > 0.7) 녀석만 '진짜 가로 실선'으로 보고 탈락시킵니다.
#     #         # 이젠 가로로 긴 "보호구역", "어린이보호구역" 글씨가 억울하게 잘리지 않습니다!
#     #         if w > 200 and solidity > 0.7:
#     #             continue

#     #         # [가드 C] 사선 점선 검거 필터 (기존 유지)
#     #         if solidity < 0.28: 
#     #             continue
            
#     #         # [가드 D] 너무 대형으로 꽉 찬 노란색 사각형 블록 검거 (기존 유지)
#     #         if area > 3000 and solidity > 0.85:
#     #             continue
            
#     #         # ----------------------------------------------------------------------
#     #         # 모든 바리케이트를 무사히 통과한 '순수 글자 뭉치'만 최종 합산
#     #         mask_crop = yellow_mask[y:y+h, x:x+w]
#     #         pure_text_pixel_count += cv2.countNonZero(mask_crop)
            
#     #         # 진짜 글자로 인정받은 구역만 주황색 사각형 박스로 디버깅 시각화
#     #         cv2.rectangle(debug_roi, (x, y), (x+w, y+h), (0, 165, 255), 2)
#     #     # print(pure_text_pixel_count)
#     #     # [4. 픽셀 밀도 기반 상태 전환(FSM) 의사결정 블록]
#     #     if pure_text_pixel_count > 5000 and self.cooldown_counter == 0:
            
#     #         if self.current_drive_mode == "NORMAL":
#     #             # self.get_logger().warn("🚨 [EVENT] 어린이 보호구역 진입! 서행 모드로 강제 전환합니다.")
#     #             self.current_drive_mode = "SCHOOL_ZONE"
#     #             self.cooldown_counter = 90  
                
#     #         elif self.current_drive_mode == "SCHOOL_ZONE":
#     #             # self.get_logger().info("🟢 [EVENT] 어린이 보호구역 해제! 정상 속도로 복귀합니다.")
#     #             self.current_drive_mode = "NORMAL"
#     #             self.cooldown_counter = 90

#     #     # [5. 현재 주행 모드에 따른 최종 속도 명령 발행]
#     #     if self.current_drive_mode == "SCHOOL_ZONE":
#     #         #자체 모터
#     #         pass
#     #         # self.publish_motor(speed=self.school_zone_speed, angle=0.0)
#     #     else:
#     #         # self.publish_motor(speed=7.0, angle=0.0)
#     #         pass

#         # 디버깅용
#         # cv2.imshow("School Zone Pure Text Filter Window", debug_roi)
#         # cv2.waitKey(1)
        
#     def publish_motor(self, speed, angle):
#         motor_msg = XycarMotor()
#         motor_msg.speed = float(speed)
#         motor_msg.angle = float(angle)
#         self.motor_pub.publish(motor_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = SchoolZoneDetector()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
    
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class SchoolZoneDetector(Node):
    """
    카메라 노면 바닥 영역(ROI)의 황색(Yellow) 픽셀 밀도를 분석하여
    어린이 보호구역 진입 시 감속하고, 해제 문구 통과 시 원래 속도로 복귀하는 노드입니다.
    """
    def __init__(self, standalone=True):
        super().__init__('school_zone_detector')
        
        self.sub = self.create_subscription(Image, '/usb_cam/image_raw/front', self.image_callback, 10)
        self.motor_pub = self.create_publisher(XycarMotor, '/xycar_motor', 10)
        self.bridge = CvBridge()
        
        # [어린이 보호구역 제어 플래그 변수]
        # "NORMAL" -> 일반 주행
        # "SCHOOL_ZONE" -> 감속 주행 모드
        self.current_drive_mode = "NORMAL"
        self.school_zone_speed = 5.0  
        self.school_zone_angle = 0.0  

        # 픽셀 감지 후 연속 오작동을 방지하기 위한 쿨다운 타임 버퍼 카운터
        self.cooldown_counter = 0 
        
        # self.get_logger().info("🧒 어린이 보호구역 바닥 감속 제어 노드가 가동되었습니다.")

    def image_callback(self, msg):
        # try:
        #     cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # except CvBridgeError as e:
        #     return
        cv_image = msg
        # 쿨다운 카운터 차감 루틴
        if self.cooldown_counter > 0:
            self.cooldown_counter -= 1

        # [1. 노면 바닥 영역 고정 ROI 지정]
        # 해상도 640x480 가정 시, 차량 보닛 바로 앞 차선 중앙 노면 영역 설정
        # 좌우 황색 차선이 최대한 밟히지 않도록 가로 폭(220~420)을 좁게 가둡니다.
        roi = cv_image[300:400, 100:500]
        
        # [2. 노면 글씨 인지용 황색(Yellow) HSV 필터링 블록]
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        # lower_yellow = np.array([15, 100, 100])
        # upper_yellow = np.array([35, 255, 255])
        lower_yellow = np.array([22, 180, 200])
        upper_yellow = np.array([32, 255, 255])
        yellow_mask = cv2.inRange(hsv_roi, lower_yellow, upper_yellow)
        
        # 노란색 노면 글자 픽셀 면적 카운트
        yellow_pixel_count = cv2.countNonZero(yellow_mask)

        # [3. 픽셀 밀도 기반 상태 전환(FSM) 의사결정 부모 블록]
        # 바닥에 적힌 큰 노란색 글자 덩어리가 박스를 통과하면 순간적으로 픽셀 수가 수천 개 이상 치솟습니다.
        # 디버깅용
        self.get_logger().info(f"노란 픽셀수 : {yellow_pixel_count}")
        if yellow_pixel_count > 6000 and self.cooldown_counter == 0:
            
            if self.current_drive_mode == "NORMAL":
                # 일반 주행 중 글자 덩어리를 만나면 -> 어린이 보호구역 진입으로 판단!
                # self.get_logger().warn("🚨 [EVENT] 어린이 보호구역 진입 감지! 감속을 수행합니다.")
                self.current_drive_mode = "SCHOOL_ZONE"
                self.cooldown_counter = 90  # 약 3초간 동일 문구 중복 인식 방지 가드 가동
                
            elif self.current_drive_mode == "SCHOOL_ZONE":
                # 이미 감속 주행 중인데 또 한 번 대형 글자 덩어리를 만나면 -> "해제" 문구 통과로 판단!
                # self.get_logger().info("🟢 [EVENT] 보호구역 해제 확인! 정상 속도로 복귀합니다.")
                self.current_drive_mode = "NORMAL"
                self.cooldown_counter = 90

        # # [4. 현재 주행 모드에 따른 최종 속도 명령 발행]
        # if self.current_drive_mode == "SCHOOL_ZONE":
        #     self.publish_motor(speed=self.school_zone_speed, angle=0.0)
        # else:
        #     self.publish_motor(speed=7, angle=0.0)

        # 디버깅 창 시각화
        # cv2.imshow("School Zone ROI Mask", yellow_mask)
        # cv2.waitKey(1)
        
    def publish_motor(self, speed, angle):
#     """ 모터 토픽 발행을 전담하는 헬퍼 함수 """
        motor_msg = XycarMotor()
        motor_msg.speed = float(speed)
        motor_msg.angle = float(angle)
        self.motor_pub.publish(motor_msg)



def main(args=None):
    rclpy.init(args=args)
    node = SchoolZoneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        


if __name__ == '__main__':
    main()