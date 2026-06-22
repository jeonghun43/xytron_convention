import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from xycar_msgs.msg import XycarMotor
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneFollowNode(Node):
    def __init__(self):
        super().__init__('line_follow_node')
        self.bridge = CvBridge()

        self.prev_steering = 0.0
        # self.publisher_steer = self.create_publisher(Float32, '/steering', 10)
        # self.publisher_throttle = self.create_publisher(Float32, '/throttle', 10)
        self.subscription = self.create_subscription(
            Image,
            '/usb_cam/image_raw/front',
            self.image_callback,
            10)
        self.motor_pub = self.create_publisher(XycarMotor, '/xycar_motor', 10)
        
        self.get_logger().info('LineFollowNode started')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = frame.shape

        # HSV 변환 및 흰색 마스크
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 25, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)
        # HSV 변환 및 노란색 마스크
        # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # lower_yellow = np.array([20, 100, 100])
        # upper_yellow = np.array([30, 255, 255])
        # mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        steering_errors = []

        # 여러 높이의 ROI에서 중심 추출
        for i in range(3):
            roi_y1 = int(height * (0.6 + i * 0.1))
            roi_y2 = int(height * (0.65 + i * 0.1))
            roi = mask[roi_y1:roi_y2, :]

            left_roi = roi[:, :width//2]
            right_roi = roi[:, width//2:]

            M_left = cv2.moments(left_roi)
            M_right = cv2.moments(right_roi)

            if M_left['m00'] > 0 and M_right['m00'] > 0:
                left_cx = int(M_left['m10'] / M_left['m00'])
                right_cx = int(M_right['m10'] / M_right['m00']) + width // 2
                lane_center = (left_cx + right_cx) // 2
                error = (lane_center - width // 2) / (width // 2)
                steering_errors.append(error)

        # 평균 steering 계산
        if steering_errors:
            avg_error = np.mean(steering_errors)
        else:
            avg_error = 0.0

        # 스티어링 필터링 (α=0.5)
        alpha = 0.5
        steering = alpha * avg_error + (1 - alpha) * self.prev_steering
        self.prev_steering = steering

        # throttle 자동 조절 (선회 시 감속)
        base_speed = 0.2
        throttle = max(0.1, base_speed - 0.2 * abs(steering))
        self.publish_motor(5.0, self.prev_steering)
    def publish_motor(self, speed, angle):
        # 모터 토픽 발행을 전담하는 헬퍼 함수
        motor_msg = XycarMotor()
        motor_msg.speed = float(speed)
        motor_msg.angle = float(angle)
        self.motor_pub.publish(motor_msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowNode()
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

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from xycar_msgs.msg import XycarMotor
# from std_msgs.msg import Float32MultiArray
# from cv_bridge import CvBridge
# import cv2
# import numpy as np

# class BevLaneTracking(Node):
#     def __init__(self):
#         super().__init__('bev_lane_tracking')

#         self.subscription = self.create_subscription(
#             Image,
#             '/usb_cam/image_raw/front',
#             self.image_callback,
#             10)
#         self.motor_pub = self.create_publisher(XycarMotor, '/xycar_motor', 10)
        
#         self.publisher_ = self.create_publisher(
#             Float32MultiArray,
#             '/lane_info',
#             10)

#         self.br = CvBridge()
#         self.get_logger().info('🦅 Bird Eye View Lane Tracking Started (Fixed Version)')

#         # 제어 상태 변수
#         self.error_x = 0.0
#         self.angle_deg = 0.0
#         self.alpha = 0.4  # LPF 상수를 살짝 높여 반응성 확보

#         # 💡 [필수 튜닝] BEV 화면(400x400)에서 왼쪽 차선과 오른쪽 차선 사이의 픽셀 거리!
#         # 대회장 바닥에 올려놓고 잰 뒤 이 값을 꼭 수정하세요. (대략 200~280 사이)
#         self.lane_width = 240.0 

#     def image_callback(self, data):
#         frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
#         height, width, _ = frame.shape

#         roi_start_y = int(height * 0.5)
#         roi = frame[roi_start_y:, :]
#         roi_h, roi_w, _ = roi.shape

#         # BEV 변환
#         src_pts = np.float32([
#             [roi_w * 0.2, roi_h * 0.1],  
#             [roi_w * 0.8, roi_h * 0.1],  
#             [roi_w * 0.05, roi_h * 0.8],  
#             [roi_w * 0.95, roi_h * 0.8]   
#         ])
#         bev_w, bev_h = 400, 400
#         dst_pts = np.float32([[0, 0], [bev_w, 0], [0, bev_h], [bev_w, bev_h]])
#         matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
#         bev_img = cv2.warpPerspective(roi, matrix, (bev_w, bev_h))

#         # HSV 흰색 필터링
#         hsv = cv2.cvtColor(bev_img, cv2.COLOR_BGR2HSV)
#         lower_white = np.array([0, 0, 180])
#         upper_white = np.array([180, 40, 255])
#         white_mask = cv2.inRange(hsv, lower_white, upper_white)

#         # 엣지 및 선 검출
#         edges = cv2.Canny(white_mask, 50, 150)
#         lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=30, minLineLength=40, maxLineGap=20)

#         bev_center = bev_w / 2.0
        
#         # 💡 1️⃣ 좌우 차선을 철저히 분리해서 담을 리스트
#         left_angles, right_angles = [], []
#         left_centers, right_centers = [], []

#         if lines is not None:
#             for line in lines:
#                 x1, y1, x2, y2 = line[0]
#                 if abs(y2 - y1) < 5: continue
                
#                 dx = x2 - x1
#                 dy = -(y2 - y1) if y2 > y1 else (y1 - y2)
#                 dx = dx if y2 > y1 else -dx
                
#                 line_angle = np.degrees(np.arctan2(dx, dy))
#                 x_center = (x1 + x2) / 2.0

#                 # 화면 중앙을 기준으로 선의 위치가 왼쪽인지 오른쪽인지 분류
#                 if x_center < bev_center:
#                     left_centers.append(x_center)
#                     left_angles.append(line_angle)
#                 else:
#                     right_centers.append(x_center)
#                     right_angles.append(line_angle)

#         raw_center_x = None
#         raw_angle = None

#         # 💡 2️⃣ 경우의 수에 따른 완벽한 중앙점(raw_center_x) 계산
#         if left_centers and right_centers:
#             # 양쪽 차선 모두 보임
#             raw_center_x = (np.mean(left_centers) + np.mean(right_centers)) / 2.0
#             raw_angle = (np.mean(left_angles) + np.mean(right_angles)) / 2.0

#         elif left_centers and not right_centers:
#             # 왼쪽 차선만 보임 -> 가상으로 오른쪽 차선을 유추하여 중앙점 계산
#             raw_center_x = np.mean(left_centers) + (self.lane_width / 2.0)
#             raw_angle = np.mean(left_angles)

#         elif right_centers and not left_centers:
#             # 오른쪽 차선만 보임 -> 가상으로 왼쪽 차선을 유추하여 중앙점 계산
#             raw_center_x = np.mean(right_centers) - (self.lane_width / 2.0)
#             raw_angle = np.mean(right_angles)

#         # 💡 3️⃣ 흔들림 방지를 위한 결합 제어 (위치 오차 + 각도 오차)
#         if raw_center_x is not None and raw_angle is not None:
#             # 위치 오차 (픽셀)
#             error_x = raw_center_x - bev_center
            
#             # 각도 제어량 (기존 방식)
#             steer_from_angle = (raw_angle / 45.0) * 100.0  
#             # 위치 제어량 (P제어: 1픽셀 틀어질 때마다 0.5씩 조향 추가, 환경에 맞게 튜닝)
#             steer_from_pos = error_x * 0.5  

#             # 두 값을 더해서 최종 조향값 산출 (차가 틀어져있으면서 위치도 벗어났다면 강하게 꺾음)
#             target_steering = steer_from_angle + steer_from_pos
            
#             # -100 ~ +100 클리핑
#             target_steering = max(min(target_steering, 100.0), -100.0)

#             # LPF 적용
#             self.angle_deg = (self.alpha * target_steering) + ((1.0 - self.alpha) * self.angle_deg)
#             self.error_x = error_x
#         else:
#             self.get_logger().warn('⚠️ 차선 완전 소실. 이전 상태 유지')

#         # 데이터 Publish 및 모터 제어
#         msg = Float32MultiArray()
#         msg.data = [float(self.error_x), float(self.angle_deg)]
#         self.publisher_.publish(msg)
        
#         self.publish_motor(5.0, self.angle_deg)

#         # 디버그 시각화
#         debug_roi = roi.copy()
#         pts = src_pts.astype(np.int32)
#         cv2.polylines(debug_roi, [pts], isClosed=True, color=(0, 255, 255), thickness=2)

#         debug_bev = bev_img.copy()
#         cv2.line(debug_bev, (int(bev_center), 0), (int(bev_center), bev_h), (255, 0, 0), 1)

#         # 현재 우리가 목표로 잡고 있는 중앙점(raw_center_x)을 노란색 동그라미로 표시
#         if raw_center_x is not None:
#             cv2.circle(debug_bev, (int(raw_center_x), int(bev_h/2)), 10, (0, 255, 255), -1)

#         if lines is not None:
#             for line in lines:
#                 x1, y1, x2, y2 = line[0]
#                 cv2.line(debug_bev, (x1, y1), (x2, y2), (0, 255, 0), 2)

#         cv2.putText(debug_bev, f"Cmd: {self.angle_deg:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
#         cv2.putText(debug_bev, f"Err X: {self.error_x:.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

#         cv2.imshow("1. ROI Warp Area", debug_roi)
#         cv2.imshow("2. BEV Result", debug_bev)
#         cv2.waitKey(1)
        
#     def publish_motor(self, speed, angle):
#         # 모터 토픽 발행을 전담하는 헬퍼 함수
#         motor_msg = XycarMotor()
#         motor_msg.speed = float(speed)
#         motor_msg.angle = float(angle)
#         self.motor_pub.publish(motor_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = BevLaneTracking()
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
# # import rclpy
# # from rclpy.node import Node
# # from sensor_msgs.msg import Image
# # from xycar_msgs.msg import XycarMotor
# # from std_msgs.msg import Float32MultiArray
# # from cv_bridge import CvBridge
# # import cv2
# # import numpy as np

# # class BevLaneTracking(Node):
# #     def __init__(self):
# #         super().__init__('bev_lane_tracking')

# #         self.subscription = self.create_subscription(
# #             Image,
# #             '/usb_cam/image_raw/front',
# #             self.image_callback,
# #             10)
# #         self.motor_pub = self.create_publisher(XycarMotor, '/xycar_motor', 10)
        
# #         self.publisher_ = self.create_publisher(
# #             Float32MultiArray,
# #             '/lane_info',
# #             10)

# #         self.br = CvBridge()
# #         self.get_logger().info('🦅 Bird Eye View Lane Tracking Started')

# #         # 제어 상태 변수
# #         self.error_x = 0.0
# #         self.angle_deg = 0.0
# #         self.alpha = 0.3  # 흔들림 방지용 LPF

# #     def image_callback(self, data):
# #         frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
# #         height, width, _ = frame.shape

# #         # 1️⃣ ROI 설정 (아래쪽 50% 영역만 사용)
# #         roi_start_y = int(height * 0.5)
# #         roi = frame[roi_start_y:, :]
# #         roi_h, roi_w, _ = roi.shape

# #         # 2️⃣ Bird Eye View (IPM: Inverse Perspective Mapping) 변환
# #         # 💡 [매우 중요] 실제 대회장 트랙에 차를 세워두고 이 4개의 점(src_pts)을 튜닝해야 합니다!
# #         # 사다리꼴 모양으로, 바닥에 있는 평행한 두 차선이 포함되도록 좌표를 잡습니다.
# #         src_pts = np.float32([
# #             [roi_w * 0.2, roi_h * 0.1],  # 좌상단
# #             [roi_w * 0.8, roi_h * 0.1],  # 우상단
# #             [roi_w * 0.05, roi_h * 0.8],  # 좌하단
# #             [roi_w * 0.95, roi_h * 0.8]   # 우하단
# #         ])

# #         # 변환될 BEV 이미지의 크기 설정
# #         bev_w, bev_h = 400, 400
# #         dst_pts = np.float32([
# #             [0, 0],          # 좌상단
# #             [bev_w, 0],      # 우상단
# #             [0, bev_h],      # 좌하단
# #             [bev_w, bev_h]   # 우하단
# #         ])

# #         # 투시 변환 행렬 계산 및 이미지 변환
# #         matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
# #         bev_img = cv2.warpPerspective(roi, matrix, (bev_w, bev_h))

# #         # 3️⃣ 흰색 차선 필터링 (HSV 기반)
# #         hsv = cv2.cvtColor(bev_img, cv2.COLOR_BGR2HSV)
# #         # 흰색은 채도(S)가 낮고, 명도(V)가 높은 특징을 가짐
# #         lower_white = np.array([0, 0, 180])
# #         upper_white = np.array([180, 40, 255])
# #         white_mask = cv2.inRange(hsv, lower_white, upper_white)

# #         # 4️⃣ 엣지 추출 및 허프 변환 (BEV 이미지 기준)
# #         edges = cv2.Canny(white_mask, 50, 150)
# #         lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=30, minLineLength=40, maxLineGap=20)

# #         bev_center = bev_w / 2
# #         angles = []
# #         x_centers = []

# #         if lines is not None:
# #             for line in lines:
# #                 x1, y1, x2, y2 = line[0]
                
# #                 # 수평선 완전 배제 (y값이 같으면 기울기 무한대 오류 방지)
# #                 if abs(y2 - y1) < 5:
# #                     continue
                
# #                 # 💡 BEV 각도 계산 (가장 직관적인 방법)
# #                 # dx, dy를 이용해 수직선(차량 직진 방향)을 기준으로 한 각도를 구합니다.
# #                 dx = x2 - x1
# #                 # y좌표는 위로 갈수록 작아지므로, 아래에서 위를 향하는 벡터로 맞춤
# #                 dy = -(y2 - y1) if y2 > y1 else (y1 - y2)
# #                 dx = dx if y2 > y1 else -dx
                
# #                 # 라디안을 디그리로 변환
# #                 # 수직선(직진)일 때 0도, 오른쪽으로 누우면 양수(+), 왼쪽으로 누우면 음수(-)
# #                 line_angle = np.degrees(np.arctan2(dx, dy))
# #                 angles.append(line_angle)

# #                 # 라인의 중간 X 좌표 (화면 중심과의 오차 계산용)
# #                 x_centers.append((x1 + x2) / 2)

# # # 5️⃣ 최종 조향각 및 오차 계산
# #         if angles and x_centers:
# #             raw_angle = np.mean(angles)
# #             raw_center_x = np.mean(x_centers)

# #             # 💡 [핵심 수정] 실제 각도를 -100 ~ 100 모터 제어값으로 매핑
# #             MAX_LANE_ANGLE = 70.0  # 차선이 45도 기울면 모터 100% 가동 (환경에 맞게 조절)
# #             target_steering = (raw_angle / MAX_LANE_ANGLE) * 100.0
            
# #             # -100 ~ 100 범위 밖으로 넘어가지 않도록 잘라내기 (Clipping)
# #             target_steering = max(min(target_steering, 100.0), -100.0)

# #             # LPF(로우패스 필터)를 적용해 값이 확 튀는 것 방지
# #             self.angle_deg = (self.alpha * target_steering) + ((1.0 - self.alpha) * self.angle_deg)
# #             self.error_x = raw_center_x - bev_center
# #         else:
# #             self.get_logger().warn('⚠️ 흰색 차선 미인식. 이전 값 유지')

# #         # 6️⃣ 데이터 Publish
# #         msg = Float32MultiArray()
# #         msg.data = [float(self.error_x), float(self.angle_deg)]
# #         self.publisher_.publish(msg)
# #         self.motor_pub.publish(XycarMotor(speed=5.0, angle=self.angle_deg))  # 예시로 속도 5 고정, 각도는 계산된 값 사용

# #         # 7️⃣ 디버그 시각화
# #         # ROI 원본에 사다리꼴(src_pts) 그리기 (튜닝용)
# #         debug_roi = roi.copy()
# #         pts = src_pts.astype(np.int32)
# #         cv2.polylines(debug_roi, [pts], isClosed=True, color=(0, 255, 255), thickness=2)
# #         cv2.putText(debug_roi, "Warp Area", (pts[0][0], pts[0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

# #         # BEV 이미지에 인식된 선 및 중심선 그리기
# #         debug_bev = bev_img.copy()
# #         cv2.line(debug_bev, (int(bev_center), 0), (int(bev_center), bev_h), (255, 0, 0), 1) # 정중앙선

# #         if lines is not None:
# #             for line in lines:
# #                 x1, y1, x2, y2 = line[0]
# #                 cv2.line(debug_bev, (x1, y1), (x2, y2), (0, 255, 0), 2)

# #         # 상태 텍스트 출력
# #         cv2.putText(debug_bev, f"Angle: {self.angle_deg:.1f} deg", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
# #         cv2.putText(debug_bev, f"Error X: {self.error_x:.1f} px", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

# #         # 화면 출력
# #         cv2.imshow("1. ROI with Trapzeoid", debug_roi)
# #         cv2.imshow("2. BEV Result", debug_bev)
# #         cv2.imshow("3. White Mask", white_mask)
# #         cv2.waitKey(1)
        
# # def publish_motor(self, speed, angle):
# # #     """ 모터 토픽 발행을 전담하는 헬퍼 함수 """
# #     motor_msg = XycarMotor()
# #     motor_msg.speed = float(speed)
# #     motor_msg.angle = float(angle)
# #     self.motor_pub.publish(motor_msg)

# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = BevLaneTracking()
# #     try:
# #         rclpy.spin(node)
# #     except KeyboardInterrupt:
# #         node.get_logger().info('종료합니다.')
# #     finally:
# #         node.destroy_node()
# #         cv2.destroyAllWindows()
# #         rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()