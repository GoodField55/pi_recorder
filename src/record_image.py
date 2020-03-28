#!/usr/bin/env python
#encoding: utf8
import rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime

class RecordImage():
    def __init__(self):
        sub = rospy.Subscriber("/cv_camera/image_raw", Image, self.get_image)
        self.bridge = CvBridge()
        self.image_org = None
        self.last_time = rospy.Time.now()

    def get_image(self,img):
        try:
            self.image_org = self.bridge.imgmsg_to_cv2(img, "bgr8")

        except CvBridgeError as e:
            rospy.logerr(e)

    def record_image(self):
        if self.image_org is None:
            return None

        org = self.image_org
        
        cv2.imwrite("/tmp/image_" + datetime.now().strftime("%Y%m%d-%H:%M:%S") +".jpg" ,org)
#        cv2.imwrite("/tmp/image.jpg" ,org)
        
        return "saved"

if __name__ == '__main__':
    rospy.init_node("record_image")
    ri = RecordImage()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        if rospy.Time.now().to_sec() - ri.last_time.to_sec() >= 10.0:
            rospy.loginfo(ri.record_image())
            ri.last_time = rospy.Time.now()

        rate.sleep()

