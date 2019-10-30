#Script to track objects using Opencv KCF
import cv2 as cv
import rospy
import numpy as np

#ROS Dependencies
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#Global variables 
label_count = 0

class track_brick:
    def __init__(self):
        self.tracker = cv.TrackerKCF_create() #Initializing KCF tracker
        #self.tracker = cv.TrackerBoosting_create()
        #self.tracker = cv.TrackerTLD_create()
        #self.tracker = cv.TrackerMedianFlow_create()
        #self.tracker = cv.TrackerGOTURN_create()
        self.tracker = cv.TrackerCSRT_create()

    def update_track(self, ori_img, tl, br):
        ori_img_1 = np.copy(ori_img)
        w = br[0] - tl[0]
        h = br[1] - tl[1]
        bbox = (tl[0], tl[1], w, h)
        winName = "Tracking"
        cv.namedWindow(winName,cv.WINDOW_NORMAL)
        cv.rectangle(ori_img_1, (tl[0],tl[1]),(br[0],br[1]),(0,0,255), 2, 1)
        cv.imshow(winName,ori_img_1)
        cv.waitKey(1)

        status = self.tracker.init(ori_img,bbox)

        while True:
            img = self.sbs_image()
            #Updating the tracker
            status, bbox = self.tracker.update(img)
            
            if(status == True):
                #Tracking success
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                cv.rectangle(img, p1 ,p2 , (0,0,255), 2 ,1)
                #Show rectangle on the image
                cv.imshow(winName,img)
                cv.waitKey(1)
            else:
                #Tracking failed
                break
 
    def sbs_image(self):
        #rospy.init_node('listener_2',anonymous=True)
        image_msg = rospy.wait_for_message('/camera/color/image_raw',Image)
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(image_msg,"bgr8")
        return cv_image

def process_track(img, tl, br):
    print("Inside process track")
    tr = track_brick()
    tr.update_track(img , tl , br)
    return 0


