# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import Float32MultiArray
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# from xycar_msgs.msg import XycarMotor

# class AdvancedYellowLineTracking(Node):
#     def __init__(self):
#         super().__init__('advanced_yellow_line_tracking')

#         self.subscription = self.create_subscription(
#             Image,
#             '/usb_cam/image_raw/front',
#             self.image_callback,
#             10)
#         self.publisher_ = self.create_publisher(
#             Float32MultiArray,
#             '/lane_info',
#             10)
#         self.motor_pub = self.create_publisher(XycarMotor, '/xycar_motor', 10)

#         self.br = CvBridge()
#         self.get_logger().info('🚀 Advanced Yellow Line Tracking Started')

#         # ✅ 흔들림 방지를 위한 부드러운 타겟값 (Low Pass Filter 용)
#         self.smoothed_center_x = None
#         # ✅ LPF 적용 비율 (0.0 ~ 1.0, 낮을수록 이전 값 유지 비중이 커서 부드러워짐)
#         self.alpha = 0.3 
#         self.error_x = 0
#         self.base_speed = 10
#         self.angle_deg = 0
#         self.max_steer_angle = 100.0

#     def image_callback(self, data):
#         frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
#         height, width, _ = frame.shape
#         cam_center = width / 2

#         if self.smoothed_center_x is None:
#             self.smoothed_center_x = cam_center

#         # 1️⃣ ROI: 멀리 보기 위해 시야 상향 조정 (상단 40%부터 밑으로)
#         roi_start_y = int(height * 0.4) 
#         roi = frame[roi_start_y:, :]
#         roi_h, roi_w, _ = roi.shape

#         # 2️⃣ 노란색 필터링 (Color)
#         hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
#         lower_yellow = np.array([22, 180, 200])
#         upper_yellow = np.array([32, 255, 255])
#         yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

#         # 3️⃣ 윤곽선 추출 (Edge)
#         gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
#         blur = cv2.GaussianBlur(gray, (5, 5), 0)
#         edges = cv2.Canny(blur, 50, 150)

#         # 💡 핵심 1: 노란색 영역 안에서만 윤곽선을 남김 (면적이 아닌 '선'만 추려냄)
#         yellow_edges = cv2.bitwise_and(edges, edges, mask=yellow_mask)

#         # 4️⃣ 허프 변환으로 진짜 선(Line) 찾기
#         lines = cv2.HoughLinesP(yellow_edges, 1, np.pi/180, threshold=20, minLineLength=30, maxLineGap=20)

#         # 💡 [추가] 허프 변환 선 검출 직전 단계에 삽입된 핵심 로직
        
#         # 1. 조향각 크기 비율 계산 (0.0 = 완전 직진 ~ 1.0 = 최대 회전 상태)
#         steer_ratio = min(abs(self.angle_deg) / self.max_steer_angle, 1.0)
        
#         # 2. 직진할 때(멀리 볼 때)와 커브 돌 때(가까이 볼 때)의 한계 비율 설정
#         min_target_y = int(roi_h * 0.2) # ROI 상단 20% (멀리 보기)
#         max_target_y = int(roi_h * 0.7) # ROI 하단 70% (바로 앞 보기)
        
#         # 3. 선형 보간(Linear Interpolation)을 통해 현재 타겟 Y 좌표 동적 결정
#         target_y = int(min_target_y + (max_target_y - min_target_y) * steer_ratio)

#         valid_x_at_target = []
        
#         # 💡 핵심 2: Lookahead Distance (멀리 있는 목표 지점)
#         # roi_h의 20% 지점(위쪽)을 목표 Y좌표로 설정하여 직진 안정성 확보
#         # target_y = int(roi_h * 0.2) 

#         if lines is not None:
#             for line in lines:
#                 x1, y1, x2, y2 = line[0]
#                 if x1 == x2: continue
                
#                 slope = (y2 - y1) / (x2 - x1)
                
#                 # 💡 핵심 3: 어린이보호구역 글씨, 횡단보도 등 수평선(기울기가 낮은 선) 무시
#                 if abs(slope) < 0.5:
#                     continue

#                 # 차선 방정식을 통해 '멀리 있는 타겟 Y'에서의 X좌표 계산
#                 intercept = y1 - slope * x1
#                 x_at_target = (target_y - intercept) / slope
                
#                 # 계산된 X가 화면 안에 있을 때만 유효한 값으로 인정
#                 if 0 <= x_at_target <= width:
#                     valid_x_at_target.append(x_at_target)

#         # 5️⃣ 목표 조향점 계산 및 필터링
#         raw_center_x = self.smoothed_center_x

#         if valid_x_at_target:
#             # 유효한 선들의 목표 X좌표 평균
#             raw_center_x = np.mean(valid_x_at_target)
            
#             # 💡 핵심 4: Low Pass Filter 적용 (왔다리 갔다리 흔들림 방지)
#             # 현재 들어온 값을 100% 믿지 않고, 이전 값에 살짝(alpha만큼만) 더해줌
#             self.smoothed_center_x = (self.alpha * raw_center_x) + ((1.0 - self.alpha) * self.smoothed_center_x)
#         else:
#             self.get_logger().warn('⚠️ 유효한 노란 선을 찾지 못함. 이전 목표점 유지.')

#         # 6️⃣ Error 계산 및 Publish
#         self.error_x = self.smoothed_center_x - cam_center
#         self.angle_deg = float(self.error_x) # 비례 제어 상수(환경에 맞게 수정)
#         self.angle_deg = max(min(self.angle_deg, 100.0), -100.0)
        
#         if self.error_x > 150 or self.error_x < -150:
#             self.base_speed = 3
#         elif self.error_x > 50 or self.error_x < -50:
#             self.base_speed = 7
#             self.angle_deg *= 0.5
#         elif self.error_x > 20 or self.error_x < -20:
#             self.base_speed = 10
#             self.angle_deg *= 0.5
#         elif self.error_x > 10 or self.error_x < -10:
#             self.base_speed = 13
#             self.angle_deg *= 0.1
#         elif -5 < self.error_x and self.error_x < 5:
#             self.base_speed = 15
#             self.angle_deg *= 0.1

#         msg = Float32MultiArray()
#         msg.data = [float(self.smoothed_center_x), float(self.error_x), float(self.angle_deg)]
#         self.publisher_.publish(msg)
#         self.publish_motor(speed=self.base_speed, angle=self.angle_deg)

#         # 7️⃣ 디버그 화면 시각화
#         debug_img = roi.copy()
        
#         # 목표 지점 Y선 (가로선 - 파란색)
#         cv2.line(debug_img, (0, target_y), (roi_w, target_y), (255, 0, 0), 1)
#         # 카메라 중앙 기준선 (세로선 - 파란색)
#         # cv2.line(debug_img, (int(cam_center), 0), (int(cam_center), roi_h), (255, 0, 0), 1)
#         # 💡 [변경] 움직이는 시선 가로선을 더 잘 보이도록 두께 조정 (1 -> 2)
#         cv2.line(debug_img, (0, target_y), (roi_w, target_y), (255, 0, 0), 2)
        
#         # 💡 [추가] 디버그 화면 좌측 상단에 현재 시선 좌표와 조향 비율 출력
#         cv2.putText(debug_img, f"Lookahead Y: {target_y} (Ratio: {steer_ratio:.2f})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
#         # 우리가 따라가는 부드러운 목표점 (빨간 동그라미)
#         cv2.circle(debug_img, (int(self.smoothed_center_x), target_y), 10, (0, 0, 255), -1)

#         # 필터링을 통과한 노란색 윤곽선 표시 (초록색)
#         if lines is not None:
#             for line in lines:
#                 x1, y1, x2, y2 = line[0]
#                 slope = (y2 - y1) / (x2 - x1 + 1e-6)
#                 if abs(slope) >= 0.5:
#                     cv2.line(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

#         cv2.imshow("Advanced Tracking", debug_img)
#         cv2.imshow("Yellow Edges", yellow_edges)
#         cv2.waitKey(1)
        
#     def publish_motor(self, speed, angle):
#         motor_msg = XycarMotor()
#         motor_msg.speed = float(speed)
#         motor_msg.angle = float(angle)
#         self.motor_pub.publish(motor_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = AdvancedYellowLineTracking()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('종료합니다.')
#     finally:
#         node.destroy_node()
#         cv2.destroyAllWindows()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
    
    
# 중심 라인은 잘 따라가는데 어린이보호구역부분이랑 나중에 노란색 선이 바깥에 갈때 문제생김
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from xycar_msgs.msg import XycarMotor
import cv2
import numpy as np

class YellowLineTrackingNode(Node):
    def __init__(self):
        super().__init__('yellow_line_tracking_node')

        self.subscription = self.create_subscription(
            Image,
            '/usb_cam/image_raw/front',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(
            Float32MultiArray,  # [line_x, error_x, angle_deg]
            '/lane_info',
            10)
        self.motor_pub = self.create_publisher(XycarMotor, '/xycar_motor', 10)

        self.br = CvBridge()
        self.get_logger().info('💛 Yellow Line Tracking Node Started2')

        # Fallback용 이전 값 저장
        self.last_line_x = None
        self.base_speed = 10

    def image_callback(self, data):
        frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        height, width, _ = frame.shape
        cam_center = width / 2

        if self.last_line_x is None:
            self.last_line_x = cam_center

        # 1️⃣ ROI 영역 지정 (차량 바로 앞 바닥을 보도록 하단 40% 설정)
        roi_start_y = int(height * 0.6)
        roi = frame[roi_start_y:, :]
        roi_h, roi_w, _ = roi.shape

        # 2️⃣ HSV 변환 및 노란색 필터링
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # 💡 대회장 조명 환경에 따라 이 Lower/Upper 범위를 튜닝해야 합니다!
        lower_yellow = np.array([22, 180, 200])
        upper_yellow = np.array([32, 255, 255])
        
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # 노이즈 제거 (모폴로지 열기 연산)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 3️⃣ 노란색 차선의 중심점(모멘트) 찾기
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        line_x = None
        
        if contours:
            # 발견된 윤곽선 중 가장 크기가 큰 것(진짜 노란 차선)을 선택
            largest_contour = max(contours, key=cv2.contourArea)
            
            # 최소 크기 조건 (노이즈 방지, 예: 픽셀 면적이 100 이상일 때만)
            if cv2.contourArea(largest_contour) > 100:
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    # 노란 차선의 중심 X 좌표 계산
                    line_x = int(M["m10"] / M["m00"])

        # 4️⃣ 예외 처리 (차선을 놓쳤을 때)
        if line_x is None:
            line_x = self.last_line_x
            self.get_logger().warn('⚠️ 노란 선을 놓쳤습니다! 이전 위치 유지 중')
        else:
            self.last_line_x = line_x

        # 5️⃣ 제어 오차(Error) 및 간이 조향각 계산
        # error_x가 (+)면 노란선이 우측에 있음 -> 우회전 필요
        # error_x가 (-)면 노란선이 좌측에 있음 -> 좌회전 필요
        self.error_x = line_x - cam_center
        
   
        # 화면 폭을 고려한 간이 조향각 계산 (P 제어 성격)
        # 예시: 오차를 각도로 환산하기 위한 비례상수 적용, 대회장 환경에 맞춰 가감
        self.angle_deg = float(self.error_x) 
        
             # 얼마나 가운데로 잘 달리고 있는지에 따른 속도 변화
        if self.error_x > 100 or self.error_x < -100:
            self.base_speed = 3
            self.angle_deg *= 0.7
        elif self.error_x > 50 or self.error_x < -50:
            self.base_speed = 7
            self.angle_deg *= 0.5
        elif self.error_x > 20 or self.error_x < -20:
            self.base_speed = 10
            self.angle_deg *= 0.4
        # elif self.error_x > 10 or self.error_x < -10:
        #     self.base_speed = 13
        #     self.angle_deg *= 0.3
        # elif -5 < self.error_x and self.error_x < 5:
        #     self.base_speed = 15
        #     self.angle_deg *= 0.1
        
        
        # 최대 조향각 제한 (예: -20도 ~ 20도)
        self.angle_deg = max(min(self.angle_deg, 100.0), -100.0)

        # 6️⃣ 데이터 발행
        msg = Float32MultiArray()
        msg.data = [float(line_x), float(self.error_x), float(self.angle_deg)]
        self.publisher_.publish(msg)
        # self.publish_motor(speed=self.base_speed, angle=self.angle_deg)
        
        
        self.get_logger().info(f"📐 Error {self.error_x} : / Angle {self.angle_deg}")
        
        
        # 7️⃣ 디버깅 시각화
        debug_img = roi.copy()
        # 카메라 중심선 (파란색)
        cv2.line(debug_img, (int(cam_center), 0), (int(cam_center), roi_h), (255, 0, 0), 2)
        # 추적한 노란선 중심 (빨간색 동그라미)
        cv2.circle(debug_img, (int(line_x), int(roi_h/2)), 15, (0, 0, 255), -1)
        
        # 텍스트 정보 표시
        cv2.putText(debug_img, f"Error: {self.error_x} / Angle : {self.angle_deg}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("Yellow Track", debug_img)
        cv2.imshow("Yellow Mask", mask)
        cv2.waitKey(1)

    # def publish_motor(self, speed, angle):
    # #     """ 모터 토픽 발행을 전담하는 헬퍼 함수 """
    #     motor_msg = XycarMotor()
    #     motor_msg.speed = float(speed)
    #     motor_msg.angle = float(angle)
    #     self.motor_pub.publish(motor_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = YellowLineTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료합니다.')
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()