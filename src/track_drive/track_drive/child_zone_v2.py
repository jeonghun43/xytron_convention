import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time

class ChildZoneDetector(Node):
    def __init__(self, standalone=True):
        super().__init__('child_zone_detector')
        # self.img_sub = self.create_subscription(Image, '/usb_cam/image_raw/front', self.image_callback, 10)
        self.motor_pub = self.create_publisher(XycarMotor, '/xycar_motor', 10)
        self.bridge = CvBridge()
        self.duplicate_count = 0
        self.zone_status = "NORMAL"
        self.school_zone_speed = 5.0
        self.standalone = standalone
        self.main_line = None
        self.cant_find_line = 0
        self.base_speed = 5
        self.angle_deg = 0.0
        self.last_lane_visible_time = 0
        
    def image_callback(self, msg):
        if self.standalone:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        else:
            cv_image = msg.copy()
        roi = cv_image[200:400, 100:500]
        
        roi_h, roi_w, _ = roi.shape
        
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([22, 180, 200])
        upper_yellow = np.array([32, 255, 255])
      
        src_points = np.float32([
            [roi_w*0.2, roi_h * 0.6],[roi_w*0.8, roi_h * 0.6],
            [roi_w*0.1, roi_h * 0.85],[roi_w*0.9, roi_h * 0.85]   
        ])
        bev_w, bev_h = 400, 400
        dst_points = np.float32([
            [0, 0],[bev_w, 0],
            [0, bev_h],[bev_w, bev_h]   
        ])
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        bev_img = cv2.warpPerspective(hsv_roi, matrix, (bev_w, bev_h))
        
        yellow_mask = cv2.inRange(bev_img, lower_yellow, upper_yellow)
        kernel = np.ones((20, 20), np.uint8)
        line_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        letter_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_TOPHAT, kernel)
        yello_count = cv2.countNonZero(letter_mask)
        if self.duplicate_count > 0:
            self.duplicate_count -= 1
        
        if yello_count > 6000 and self.duplicate_count == 0:
            if self.zone_status == "NORMAL":
                self.zone_status = "SCHOOL_ZONE"
                # print("SCHOOL ZONE")
            elif self.zone_status == "SCHOOL_ZONE":
                self.zone_status = "NORMAL"
                # print("NORMAL ZONE")
            self.duplicate_count = 30        
        
        # $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
        if self.zone_status == "SCHOOL_ZONE":
            # print("sch")
            look_ahead_y_start = 80
            look_ahead_y_end = 120
            
            # 2. 좌우 외곽 실선 차선을 배제하기 위해 X축 중심 반경도 제한합니다 (중앙 점선 타겟팅)
            # 화면 정중앙(200) 기준 좌우 70픽셀 범위만 타겟으로 잡습니다. (X: 130 ~ 270)
            search_x_start = 130
            search_x_end = 270
            
            # 주시 영역 크롭
            target_roi = line_mask[look_ahead_y_start:look_ahead_y_end, search_x_start:search_x_end]
            
            # 3. 해당 영역 내에서 흰색(노란색 마스크 처리된) 픽셀들의 인덱스를 추출합니다.
            pixel_y, pixel_x = np.where(target_roi == 255)
            
            # 4. 검출된 노란색 픽셀이 존재한다면 무게중심(평균 X값)을 구합니다.
            if len(pixel_x) > 0:
                # 크롭된 영역 기준이므로 원본 BEV 좌표계로 복원하기 위해 search_x_start를 더해줍니다.
                center_of_yellow_x = int(np.mean(pixel_x)) + search_x_start
                
                target_x = 245
                diff_x = center_of_yellow_x - target_x
                
                # 조향 감도 파라미터 (차가 너무 대기 주기가 느리거나 둔하면 1.2 ~ 1.5 정도로 키우세요)
                steering_gain = 1.0
                self.angle_deg = np.clip(diff_x * steering_gain, -100, 100)
                
                # 라인 가시 시간 업데이트
                self.last_lane_visible_time = time.time()
                self.base_speed = 5
            # 독립 구동 모드 시 모터 명령 송신
            if self.standalone:
                self.publish_motor(self.base_speed, self.angle_deg)
            self.publish_motor(self.base_speed, self.angle_deg)
            # -----------------------------------------------------------------
            # 디버깅 시각화 (눈으로 직접 트래킹 지점을 확인해보세요)
            # -----------------------------------------------------------------
            # debug_img = cv2.cvtColor(line_mask, cv2.COLOR_GRAY2BGR)
            # # 검색 영역 표시 (초록색 상자)
            # cv2.rectangle(debug_img, (search_x_start, look_ahead_y_start), (search_x_end, look_ahead_y_end), (0, 255, 0), 2)
            
            # if len(pixel_x) > 0:
            #     # 계산된 무게중심점 표시 (빨간색 점)
            #     cv2.circle(debug_img, (center_of_yellow_x, int((look_ahead_y_start + look_ahead_y_end)/2)), 5, (0, 0, 255), -1)
                
            # cv2.imshow("Centroid Tracking Debug", debug_img)
            # cv2.waitKey(1)
            
            # cv2.imshow("line", roi)
            # cv2.waitKey(1)
        # print(yello_count)
        # print(self.zone_status)
        # print(self.duplicate_count)
        
        #width 480, height 640
        #0.2 = 96,  0.52= 333
        # cv2.line(roi, (int(roi_w*0.2), int(roi_h * 0.52)), (int(roi_w*0.8), int(roi_h * 0.52)), (0, 0, 255), 2)
        # cv2.line(roi, (int(roi_w*0.05), int(roi_h * 0.8)), (int(roi_w*0.95), int(roi_h * 0.8)), (0, 0, 255), 2)
        # cv2.imshow("roi", roi)
        # debug_roi = cv2.cvtColor(yellow_mask, cv2.COLOR_GRAY2BGR)
        # cv2.imshow("debug_roi", debug_roi)
        # cv2.imshow("line_mask", line_mask)
        # cv2.imshow("letter_mask", letter_mask)
        # cv2.imshow("bev_img", bev_img)
        # cv2.waitKey(1)
        
    def publish_motor(self, speed, angle):
        motor_msg = XycarMotor()
        motor_msg.speed = float(speed)
        motor_msg.angle = float(angle)
        self.motor_pub.publish(motor_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ChildZoneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()

        

        