#!/usr/bin/env python
#encoding: utf8
import rospy, cv2
import glob
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime


#
#  switch0 入力が high の時、下記画像記録機能が動作する
#  switch0 入力が low になっても　300sec 間は記録を続ける
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
        self.dev_switch0 = "/dev/rtswitch0"

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
        rospy.loginfo("saved. file_num=%d , %s\n",self.file_num, fn)
        return "saved"

    def record_image_with_judge(self):
        if rospy.Time.now().to_sec() - self.last_time.to_sec() >= 10.0:   # 10秒毎に撮影
            self.record_image()
            self.last_time = rospy.Time.now()

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
        #rospy.loginfo("num=%d\n", self.file_num)
        self.num = 0

    def monitor_movie(self):
        if self.file_num > 0:
            try:
                #rospy.loginfo("num=%d , file_num=%d , %s\n",self.num, self.file_num, self.file_list[self.num])
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

        else:
            self.get_file_list()

    def disp_info(self):
        rospy.loginfo("num=%d , file_num=%d , %s\n",self.num, self.file_num, self.file_list[self.num])

    def get_switch0(self):
        try:
            with open(self.dev_switch0,"r") as f:
                data = f.readline()
                #rospy.loginfo("date=%s\n", data)
                return int(data)
        except IOError:
            rospy.logerr("can't read from " + self.dev_switch0)
            return 999

if __name__ == '__main__':
    rospy.init_node("record_image")
    ri = RecordImage()
    ri.get_file_list()

    rate = rospy.Rate(10)
    switch0_old = 0
    switch0_flag = False
    switch0_low_edge_time = rospy.Time.now()    # dummy

    while not rospy.is_shutdown():
        sw0 = ri.get_switch0()
        #rospy.loginfo("sw0 = %d\n", sw0)

        if sw0 == 1:     # switch0 port = high
            ri.record_image_with_judge()
            switch0_old = 1
        else:                           # switch0 port = low
            if switch0_old == 1:      # switch0 high to low edge
                switch0_low_edge_time = rospy.Time.now()
                switch0_old = 0
                switch0_flag = True     # true during low edge to 300sec

            if switch0_flag:            # switch0 port is low within 300sec
                if rospy.Time.now().to_sec() - switch0_low_edge_time.to_sec() <= 300.0:   # switch0 off から５分間撮影
                    ri.record_image_with_judge()
                else:
                    switch0_flag = False    # switch0 port is low over 300sec

        ri.monitor_picture()
        ri.monitor_movie()

        rate.sleep()
    
    ri.disp_info()

