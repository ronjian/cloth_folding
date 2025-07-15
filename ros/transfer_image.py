#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from unity_robotics_demo_msgs.msg import Jpg
import cv2
from cv_bridge import CvBridge
import rosgraph
import time

class ImageToJPG:
    def __init__(self):
        rospy.init_node('transfer_image', anonymous=True)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Publisher for the JPG data
        self.jpg_pub = rospy.Publisher('/jpg', Jpg, queue_size=10)
        
        # Subscriber to the RGB image topic
        self.image_sub = rospy.Subscriber('/rgb', Image, self.image_callback)
        
        rospy.loginfo("Image to JPG converter node initialized")
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Encode the image as JPEG
            ret, jpg_data = cv2.imencode('.jpg', cv_image)
            
            if ret:
                jpg_msg = Jpg()
                jpg_msg.data = jpg_data.tobytes()
                # self.wait_for_connections(self.jpg_pub, '/jpg')
                self.jpg_pub.publish(jpg_msg)
            else:
                rospy.logwarn("Failed to encode image as JPG")
                
        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))
    
    def wait_for_connections(self, pub, topic):
        ros_master = rosgraph.Master('/rostopic')
        topic = rosgraph.names.script_resolve_name('rostopic', topic)
        num_subs = 0
        for sub in ros_master.getSystemState()[1]:
            if sub[0] == topic:
                num_subs+=1

        for i in range(10):
            print(pub.get_num_connections(), num_subs)
            if pub.get_num_connections() == num_subs:
                return
            time.sleep(0.1)
        raise RuntimeError("failed to get publisher")


if __name__ == '__main__':
    try:
        converter = ImageToJPG()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass