# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from my_yolo_wrapper_interfaces.msg import BoundingBox 

class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.depth_sub = self.create_subscription(
      Image, 
      '/zed2i/zed_node/depth/depth_registered', 
      self.depth_callback, 
      10)
    self.depth_sub # prevent unused variable warning

    self.boundingbox_sub = self.create_subscription(
      BoundingBox, 
      '/my_yolo/BoundingBox', 
      self.boundingbox_callback, 
      10)
    self.boundingbox_sub # prevent unused variable warning
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    # Declare boundingbox message 
    self.boundingbox=BoundingBox()

  def boundingbox_callback(self, boundingbox):
    """
    Callback function.
    """
    # Save data
    self.boundingbox = boundingbox

   
  def depth_callback(self, depth_frame):
    """
    Callback function.
    """
    # print(type(depth_frame.data))
    fwidth=depth_frame.width
    fheight=depth_frame.height
    u=int(fwidth/2)
    v=int(fheight/2)


    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(depth_frame, desired_encoding="32FC1")

    print("coordintate",u+fwidth*v)
    print("depth", current_frame[u][v])
    # print("fwidth: {}, fheight: {}".format(fwidth,fheight))

    if self.boundingbox.detected == 1:
      
      xmin=self.boundingbox.xmin
      xmax=self.boundingbox.xmax
      ymin=self.boundingbox.ymin
      ymax=self.boundingbox.ymax

      # Draw rectangle using bounding box
      cv2.rectangle(current_frame, (xmin, ymin), (xmax, ymax), (255,255,255), 10)
      # 이미지에서 원하는 영역을 자르기 위해서는 [ startY:endY, startX:endX ] 형태로 Array slicing 합니다.
      cv2.imshow("cropped depth", current_frame[ymin:ymax, xmin:xmax])
    
    cv2.circle(current_frame, (u,v), 5, (255,255,255))
    # Display image
    cv2.imshow("depth_frame", current_frame)
    cv2.waitKey(1)
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()