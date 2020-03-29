#!/usr/bin/env python
#encoding: utf8
import rospy, cv2
import glob
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime


#
#　USBカメラの撮影画像を　１０秒毎に　/mytemp/ ディレクトリに保する
#　撮影中の動画および記録画像は、下記URLで確認できる
#
# http://192.168.1.***:10000/stream?topic=cv_camera/image_raw  -> USBカメラ撮影画像（リアルタイム）
#                     :10000/stream?topic=/picture             -> 記録画像（パラメータで指定）
#                         rosparam set /picture_name "/image_20200329-11:05:45.jpg"
#                     :10000/stream?topic=/movie               -> 記録画像を昇順に再生
#


class RecordImage():
    def __init__(self):
        self.pub = rospy.Publisher("picture", Image, queue_size=1)
        self.pub2 = rospy.Publisher("movie", Image, queue_size=1)
        self.sub = rospy.Subscriber("/cv_camera/image_raw", Image, self.get_image)
        self.bridge = CvBridge()
        self.image_org = None
        self.last_time = rospy.Time.now()
        self.last_picture = ""
        self.file_err_flag = False      # for picture
        self.dir = "/mytemp/"
        self.file_list = []
        self.file_num = 0
        self.num = 0

    def get_image(self,img):
        try:
            self.image_org = self.bridge.imgmsg_to_cv2(img, "bgr8")

        except CvBridgeError as e:
            rospy.logerr(e)

    def record_image(self):
        if self.image_org is None:
            return None

        org = self.image_org
        fn = self.dir + "image_" + datetime.now().strftime("%Y%m%d-%H:%M:%S") +".jpg"        
        cv2.imwrite(fn ,org)
#        rospy.loginfo(fn)

        return "saved"

    def monitor_picture(self):
        picture_name = self.dir + rospy.get_param('picture_name',"image.jpg")
        
        if self.last_picture != picture_name:
            self.file_err_flag = False

        if self.file_err_flag == False:
            self.last_picture = picture_name
            try:
                img = cv2.imread(self.last_picture)
                self.pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))        
                rospy.loginfo(self.last_picture)
            except:
                self.file_err_flag = True   # 一度読み込みエラー起こしたら２回目以降は読み込みしない
                rospy.logerr("can't open image file(%s)", self.last_picture)

    def get_file_list(self):
        self.file_list = sorted(glob.glob('/mytemp/*.jpg'))
        self.file_num = len(self.file_list)
        rospy.loginfo("num=%d\n", self.file_num)
        self.num = 0

    def monitor_movie(self):
        try:
            rospy.loginfo("num=%d , file_num=%d , %s\n",self.num, self.file_num, self.file_list[self.num])
            img2 = cv2.imread(self.file_list[self.num])
            self.pub2.publish(self.bridge.cv2_to_imgmsg(img2, "bgr8"))
            self.num += 1
            if self.num == 112: self.num = 113  # self.num=112 の時、imread エラーとなるため
                # terminate called after throwing an instance of 'std::out_of_range'
                #  what():  basic_string::substr: __pos (which is 140) > this->size() (which is 0)

            if self.num >= self.file_num :
                #rospy.loginfo("reset_num num=%d , file_num=%d\n",self.num, self.file_num)
                self.get_file_list()

        except:
            rospy.logerr("can't open image file(%s)", self.file_list[self.num])


if __name__ == '__main__':
    rospy.init_node("record_image")
    ri = RecordImage()
    ri.get_file_list()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        if rospy.Time.now().to_sec() - ri.last_time.to_sec() >= 10.0:
            rospy.loginfo(ri.record_image())
            ri.last_time = rospy.Time.now()

        ri.monitor_picture()
        ri.monitor_movie()

        rate.sleep()

