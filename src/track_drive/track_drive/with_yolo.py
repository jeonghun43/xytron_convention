import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO
import cv2


class with_yolo(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/usb_cam/image_raw/front', self.image_callback, 10)
        self.get_logger().info("✅ YOLOv8 ROS2 Detector Started")
        
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        prediction = self.model(frame)
        result = prediction[0].plot()
        cv2.imshow("YOLO Detection", result)
        cv2.waitKey(1)
        
    # 신호등만 보이게 하기
    # def image_callback(self, msg):
    #     try:
    #         frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    #     except:
    #         self.get_logger().error("Error .....")
    #         return
            
    #     prediction = self.model(frame, conf=0.3, verbose=False)
    #     display_frame = frame.copy()
        
    #     boxes = prediction[0].boxes
    #     for box in boxes:
    #         class_id = int(box.cls[0])
    #         confidence = float(box.conf[0])
            
    #         # 오직 신호등만 필터링하여 드로잉 루프 가동
    #         if class_id == 9:
    #             xmin, ymin, xmax, ymax = map(int, box.xyxy[0])
                
    #             # 선명한 하늘색 박스 하이라이팅
    #             cv2.rectangle(display_frame, (xmin, ymin), (xmax, ymax), (255, 255, 0), 2)
                
    #             text = f"Signal ({confidence*100:.0f}%)"
    #             cv2.putText(display_frame, text, (xmin, ymin - 10), 
    #                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        
    #     cv2.imshow("Only Traffic Light Display", display_frame)
    #     cv2.waitKey(1)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = with_yolo()
    rclpy.spin(node)
    node.destory_node()
    rclpy.shutdown()
    
if __name__ == main:
    main()