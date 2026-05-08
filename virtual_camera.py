import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VirtualCamera(Node):
    def __init__(self):
        super().__init__('virtual_camera_node')
        # Creiamo il TUBO (Topic) su cui viaggeranno le immagini
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Un timer che scatta 30 volte al secondo (30 FPS)
        self.timer = self.create_timer(0.033, self.timer_callback) 
        
        # Carichiamo il video appena scaricato
        self.cap = cv2.VideoCapture('dashcam.mp4')
        self.bridge = CvBridge() # Il traduttore magico tra OpenCV e ROS
        self.get_logger().info("Trasmittente online: sto pubblicando il video stradale su /camera/image_raw...")

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        # Se il video finisce, lo facciamo ripartire da capo (Loop)
        if not ret:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()
            
        if ret:
            # Rimpiccioliamo un po' il video per non appesantire la rete
            frame = cv2.resize(frame, (640, 480))
            
            # TRADUZIONE: da immagine BGR (OpenCV) a Messaggio ROS 2
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            
            # SPARIAMO IL MESSAGGIO NEL TOPIC!
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VirtualCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
