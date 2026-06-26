# _________________________________________________________
# 통합용v1
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from xycar_msgs.msg import XycarMotor
import cv2, time
import numpy as np

class LineTraceNode(Node):
    def __init__(self, standalone=True):
        super().__init__('line')
        # self.is_printed = False ##디버깅용
        if standalone:
            self.image_sub = self.create_subscription(Image, '/usb_cam/image_raw/front', self.image_callback, 10) #개별 실행용
        self.motor_pub = self.create_publisher(XycarMotor, "/xycar_motor", 10)
        self.standalone = standalone
        self.bridge = CvBridge()
        # self.get_logger().info(" 🛣️ Line trace node has started.")
        self.main_line = None
        self.fast = False
        self.base_speed = 12
        self.angle_deg = 0.0
        self.cant_find_line = 0
        
        self.alpha = 0.8
        self.standard_d = 520
        
        self.last_lane_visible_time = time.time()
        
    def image_callback(self, data):
        # print("line?")
        if self.standalone:
            try:
                frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                return
        else:
            frame = data
        
        height, width, _ = frame.shape
        center_x = width / 2
        left_lane_bucket = []
        right_lane_bucket = []
        roi_start_y = int(height * 0.3)
        roi = frame[roi_start_y:, :]
        roi_h, roi_w, _ = roi.shape

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 230])
        upper_white = np.array([180, 5, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        lines = cv2.HoughLinesP(mask, 1, np.pi / 180, threshold=50, minLineLength=30, maxLineGap=40)
        
        # debug_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        # cv2.imshow("mask Image", debug_mask)
        # cv2.waitKey(1)
        # 한번만 실행
        # if lines is not None and not self.is_printed:
        #     print(sorted_lines.shape)
        #     print(sorted_lines)
        #     self.is_printed = True
        if lines is None:
            
            if time.time() - self.last_lane_visible_time < 0.2:
                self.base_speed = 10  # 정상 속도로 뻐기기
            else:
                self.base_speed = 1   # 0.2초 이상 안 보이면 진짜 위험하니까 감속
            return
        else :
            self.last_lane_visible_time = time.time()
 
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            # 수직선 분모 0 에러 예외 처리 가드
            if x2 - x1 == 0: 
                continue
                
            # [2. 1단계 필터: 선분의 수학적 기울기(Slope) 계산]
            slope = (y2 - y1) / (x2 - x1)
            
            # 완만하게 누워있는 가로선(노이즈, 출발선 등)은 기본 기울기 컷오프(0.2)로 1차 필터링
            if abs(slope) < 0.2:
                continue

            # [3. 2단계 필터: 나와 가장 가까운 점(최하단 점)의 X 좌표 포착]
            # OpenCV는 Y값이 아래로 갈수록 커지므로, y1과 y2 중 더 큰 값을 가진 쪽이 내 차와 가장 가까운 점입니다.
            if y1 > y2:
                bottom_x = x1
            else:
                bottom_x = x2

            # ----------------------------------------------------------------------
            # [4. 기울기 부호와 하단 점의 위치를 결합한 2중 조건 검증 블록]
            # ----------------------------------------------------------------------
            if slope < 0 and bottom_x < center_x:
                left_lane_bucket.append([x1, y1, x2, y2])
                
            elif slope > 0 and bottom_x > center_x:
            # elif bottom_x > center_x:
                right_lane_bucket.append([x1, y1, x2, y2])
            # cv2.line(roi, (x1,y1), (x2, y2), (0,255,0), 3)    
           
        for line in left_lane_bucket:
            x1, y1, x2, y2 = line
            cv2.line(roi, (x1, y1), (x2, y2), (0, 0, 255), 3)

        for line in right_lane_bucket:
            x1, y1, x2, y2 = line
            cv2.line(roi, (x1, y1), (x2, y2), (0, 255, 0), 3)
            
        left_array = np.array(left_lane_bucket)
        left_array = left_array.reshape(-1, 2)
        left_sort_indices = np.argsort(left_array[:, 1])[::-1]
        left_sorted = left_array[left_sort_indices]
        
        right_array = np.array(right_lane_bucket)
        right_array = right_array.reshape(-1, 2)
        right_sort_indices = np.argsort(right_array[:, 1])[::-1]
        right_sorted = right_array[right_sort_indices]
        if left_sorted.size != 0 and right_sorted.size != 0:
            if left_sorted[0, 0] > right_sorted[0, 0]: 
                self.main_line = 'left'
            else:
                self.main_line = 'right'
            
        x, y = None, None
        if self.main_line == 'left':
            x = left_sorted[:, 0]
            y = left_sorted[:, 1]
        elif self.main_line == 'right':
            x = right_sorted[:, 0]
            y = right_sorted[:, 1]
        else:
            # self.get_logger().warning('주 차선이 없습니다')
            return
    
        if x.size < 2 or y.size < 2:
            # self.get_logger().warning('차선 픽셀이 부족하여 현재 프레임을 건너뜁니다.')
            self.base_speed = 5
            if self.angle_deg > 0:
                self.angle_deg = 100
            elif self.angle_deg < 0:
                self.angle_deg = -100
            self.cant_find_line = 45
            self.publish_motor(self.base_speed, self.angle_deg)
            return

        try:
            coefficient = np.polyfit(y, x, 1)
            calculate_x = np.poly1d(coefficient)
            
            diff_x = calculate_x(200) - self.standard_d
            # print(abs(calculate_x(50) - calculate_x(200)))
            if (200 < abs(calculate_x(50) - calculate_x(200)) < 230) and (-20 < self.angle_deg < 20):
                self.fast=True
            else:
                self.fast=False
            
            #우측 정렬 실험
            # diff_x = calculate_x(200) - 500
            # print(calculate_x(200))
            
            # if(diff_x > 20 or diff_x < -20):
            #     diff_x = 0
            
            self.angle_deg = np.clip(diff_x * self.alpha, -100, 100)
            
            if self.cant_find_line == 0:
                if self.fast:
                    self.base_speed = 17
                    self.alpha = 0.6
                else:
                    self.base_speed = 12
                    self.alpha = 0.9
            elif self.cant_find_line > 0:
                self.cant_find_line -= 1
            # 자체 모터
            self.publish_motor(self.base_speed, self.angle_deg)
            
            # 디버깅용
            # print("cal_x: ", calculate_x(200))
        except (TypeError, ValueError, np.linalg.LinAlgError) as exc:
            # self.get_logger().warning(f'차선 직선 계산 실패: {exc}')
            return
        # cv2.line(roi, (320, 0), (320, roi_h), (255, 0, 0), 2)  # 화면 중앙 세로선
        #디버깅용
        # cv2.imshow("hough", roi)
        # # cv2.imshow("mask Image", mask)
        # cv2.waitKey(1)
        
        # 디버깅용
        # if self.is_printed == False:
        #     self.is_printed = True
        #     print('frame shape: ', frame.shape)
        #     print("left_sorted: ", left_sorted)
        #     print("______________________________")
        #     print("right_sorted: ", right_sorted)
            
        #     print("cal_x: ", calculate_x(200))
    def publish_motor(self, speed, angle):
    #     """ 모터 토픽 발행을 전담하는 헬퍼 함수 """
        # print(f"💌 [MOTOR OUT] Speed Command: {speed} | Angle: {angle}")
        motor_msg = XycarMotor()
        motor_msg.speed = float(speed)
        motor_msg.angle = float(angle)
        self.motor_pub.publish(motor_msg)
           
        
def main(args=None):
    rclpy.init(args=args)
    line = LineTraceNode()
    rclpy.spin(line)
    line.destroy_node()
    rclpy.shutdown()
     
if __name__ == '__main__':
    main()
