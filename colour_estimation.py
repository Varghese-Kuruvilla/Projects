#Converting images into YUV colourspace
#!/usr/bin/env python
import sys
sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
import rospy
#from std_msgs.msg import String
import cv2 as cv
import numpy as np
import glob
import matplotlib.pyplot as plt
from statistics import mean
import time
import pickle
#import pandas as pd

#ROS Dependencies
from std_msgs.msg import Float32
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#Importing functions from other modules
from pose_est_theta import pose_estimation
from track_own import track_id
#from kalman_filter import tracker_kf 


#Global variables
area_anomaly = [] #List for detection of area anomalies
#These values should be ideally read from the rostopic
#fx = 530
#fy = 530
fx = 617.08
fy = 617.77
cx = 332.02
cy = 240.98


#For testing
#read_text = pd.read_csv("/home/varghese/data_12th_Sept/2019-09-11-18-53-27/data_analyze.txt")
#read_count = 0
#dict_debug = {} #First entry is lidar depth and second entry is the depth calculated from monocular camera
#debug_lidar_depth = 0
#debug_true_depth = 0

#Initializing video writer
out = cv.VideoWriter(
        "output.avi", cv.VideoWriter_fourcc(*"MJPG"), 10.0,
        (640, 480))


class contour_process:

    def __init__(self):
       self.img_yuv = 0
       self.hist = 0
       self.v_channel = 0
       self.lower_thresh = 0
       self.upper_thresh = 0
       self.mask = 0
       self.mask_erode = 0
       self.kernel_erode = 0
       self.cnt = 0
       self.box_all = []

    def clear_all(self):

       self.img_yuv = 0
       self.hist = 0
       self.v_channel = 0
       self.lower_thresh = 0
       self.upper_thresh = 0
       self.mask = 0
       self.mask_erode = 0
       self.kernel_erode = 0
       self.cnt = 0
       self.box_all = []

       
    def validate_cnt(self,cnt):
        global fy
        #global read_count
        #global debug_lidar_depth
        #read_count = read_count + 1
        threshold = 50  #The threshold should be made adaptive
        depth = subscribe_depth()
        depth.data = depth.data - 100  #To compensate for the error
        print("depth inside colour estimation:",depth.data)
        #if(read_count == 1):
        #    debug_lidar_depth = depth.data - 20
        perimeter_cnt = cv.arcLength(cnt,True)
        print("perimeter_cnt:",perimeter_cnt)
        pixel_length = (fy * 30)/(depth.data)
        pixel_breadth = (fy * 20)/(depth.data)

        perimeter_depth = 2 * (pixel_length + pixel_breadth)
        print("perimeter_depth:",perimeter_depth)

#        pixel_length_1 = (fy * 30)/(depth.data -110)
#        pixel_breadth_1 = (fy * 20)/(depth.data - 110)
#

        if(abs(perimeter_cnt - perimeter_depth)<=threshold):
            return 1 #Indicates that the contour should be considered for further processing
        else:
            return 0
        

    def find_slope(self,seg,img,hist,v_channel,mask,mask_erode):
        box_all = [] #List of np arrays which contains box coordinates
        #seg is the segmented image
        #img is the original image on which we can draw the contours
                                  #hist is only for debugging purposes
        # draw_cnt = segs
        #img_approx = np.copy(img)
        #img_rect = np.copy(img)
        #img_cnt = np.copy(img)


        #Finding out the contours
        self.cnt , hierarchy = cv.findContours(seg , cv.RETR_TREE , cv.CHAIN_APPROX_SIMPLE)
        #print("hierarchy:",hierarchy)
        #print("len(cnt):",len(cnt))



        #Drawing contours on the image    
        #cv.drawContours(img_cnt , self.cnt , -1 , (0,255,0) , 3)
        #winName_cnt = "Drawing contours"
        #cv.namedWindow(winName_cnt,cv.WINDOW_NORMAL)
        #cv.imshow(winName_cnt,img_cnt)
        #cv.waitKey(0)


        #So if the hierarchy of [i][3] is zero we fit a rotated rect to that particular contour
        #winName_rect = "Rotated rectangle"
        #cv.namedWindow(winName_rect,cv.WINDOW_NORMAL)

        global read_count
        for i in range(0, len(hierarchy[0])):
            if(hierarchy[0][i][3] == 0):
                #Validating the contours using area considerations
                flag_valid_cnt = self.validate_cnt(self.cnt[i])
                if(flag_valid_cnt == 1):
                    rect = cv.minAreaRect(self.cnt[i])
                    #print("rect:",rect)
                    box = cv.boxPoints(rect)
                    box = np.int0(box)
                    self.box_all.append(box)

                #cv.drawContours(img_rect,[box],0,(0,0,255),2)
                #cv.imshow(winName_rect,img_rect)
                #cv.waitKey(0)
        read_count = 0

        if(len(self.box_all) >=1):
            return self.box_all , 1
        
        else:
            return 0,0
        


        #Fitting a polygon to the contour using approx polyDP
       # len_cnt = len(cnt)
       # epsilon = 0.1 * cv.arcLength(cnt[len_cnt-1],True)
       # approx_cnt = cv.approxPolyDP(cnt[len_cnt-1],epsilon,True)
       # rect = cv.minAreaRect(approx_cnt)
       # print("rect:",rect)
       # print("rect[0][0]:",rect[1][0])
       # print("rect[0][1]:",rect[1][1])
       # area_rect = rect[1][0] * rect[1][1]
       # #Finding out the average area of the list
       # if(len(area_anomaly) >= 1):
       #     area_mean = mean(area_anomaly)
       #     if((area_rect > area_mean + (0.75*area_mean)) or (area_rect) < (area_mean - (0.75*area_mean))):
       #             print("Area anomaly detected!!")
       #             print("area_anomaly:",area_anomaly)
       #             return 0 , 0 #Indicates error
       #             #inp = input("Waiting for input..")
       #             #We will proceed to discard the value associated with this area
       #     else:
       #         area_anomaly.append(area_rect)
       # else:
       #     area_anomaly.append(area_rect)
       # if(len(area_anomaly) == 10):
       #     area_anomaly.pop(0)




    def colour_analyse(self,img):
        # winName = "Initial segment"
        # cv.namedWindow(winName , cv.WINDOW_NORMAL)
        # cv.imshow(winName , img)
        # cv.waitKey(0)

        #Analysing in YUV channel
        img_yuv = cv.cvtColor(img , cv.COLOR_BGR2YUV)
        self.v_channel = img_yuv[:,:,2]

        #winName_1 = "V channel"
        #cv.namedWindow(winName_1 , cv.WINDOW_NORMAL)
        #cv.imshow(winName_1 , self.v_channel)
        #cv.waitKey(0)

        #Calculating and plotting histograms
        #plt.title("Histogram analysis")

        #Simple thresholding
        #v_channel = cv.medianBlur(v_channel,11)
        ##v_channel = cv.bilateralFilter(v_channel,9,75,75)
        #th = cv.adaptiveThreshold(v_channel,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY,11,2)
        #winName = "Adaptive thresholding"
        #cv.namedWindow(winName,cv.WINDOW_NORMAL)
        #cv.imshow(winName,th)
        #cv.waitKey(0)
        
        self.hist = cv.calcHist([self.v_channel],[0],None,[256],[0,256])
        
        #plt.plot(self.hist)
        #plt.xlim([0,256])
        #plt.show()


        
        #Calculating the variance
        #std_v = np.std(v_channel)
        #print("std_v:",std_v)
        std_v = 33.0   #This should be calculated dynamically

        mean_v = np.mean(self.v_channel)
        #print("mean_v:",mean_v)

        mean = np.where(self.hist == np.amax(self.hist)) #Mean is considered as the maximum point in the histogram
        #print("mean:",mean)
        #lower_thresh_1 = 0
        #print("lower_thresh_1:",lower_thresh_1)
        #upper_thresh_1 = mean[0][0] - (std_v)
        #lower_thresh_2 = mean[0][0] + (std_v)
        #upper_thresh_2 = 255

        self.lower_thresh = mean[0][0] - (std_v)
        self.upper_thresh = mean[0][0] + (std_v)
        #print("lower_thresh:",lower_thresh)
        #print("upper_thresh:",upper_thresh)

        #Applying otsu thresholding
        #ret , thresh = cv.threshold(v_channel , 0 , 255 , cv.THRESH_BINARY + cv.THRESH_OTSU)
        #print("Otsu threshold:",ret)

        self.mask = cv.inRange(self.v_channel , self.lower_thresh , self.upper_thresh)

        #winName_3 = "Mask_Image"
        #cv.namedWindow(winName_3,cv.WINDOW_NORMAL)
        #cv.imshow(winName_3,mask)
        #key = cv.waitKey(0)
        #if(key & 0xFF == ord('q')):
        #    cv.destroyAllWindows()


        #The mask contains some holes, try erosion
        self.kernel_erode = np.ones((3,3),np.uint8)
        self.mask_erode = cv.erode(self.mask,self.kernel_erode,iterations = 1)
        

        #winName_4 = "Result of erosion"
        #cv.namedWindow(winName_4,cv.WINDOW_NORMAL)
        #cv.imshow(winName_4,mask_erode)
        #cv.waitKey(0)

        #seg = cv.bitwise_and(img , img , mask=mask_erode)
        #seg = np.zeros_like(v_channel, np.uint8)
        #seg[mask] = np.copy(v_channel[mask])




        #winName_2 = "Segmented Image"
        #cv.namedWindow(winName_2,cv.WINDOW_NORMAL)
        #cv.imshow(winName_2,seg)
        #cv.waitKey(0)
        ##print("hist:",hist)
     
        box_all , flag = self.find_slope(self.mask_erode,img,self.hist,self.v_channel,self.mask,self.mask_erode) #hist and v_channel are for debugging

        if(flag == 1):
            return box_all , flag
        else:
            return 0,0

        #return 1,1


    #def process_colour(img):
    #    approx_cnt , flag = colour_analyse(img)
    #    return approx_cnt , flag #If flag = 0 then it indicates an error


def subscribe_depth():
    depth = rospy.wait_for_message('/lidar',Float32)
    return depth



def subscribe_image():
    rospy.init_node('listener_1',anonymous=True)
    #img = rospy.Subscriber('/camera/color/image_raw',Image,callback_image)
    #rospy.spin()
    image_msg = rospy.wait_for_message('/camera/color/image_raw',Image)
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_msg,"bgr8")
    return cv_image
    
    #return img

#def callback_depth(data):
#    global flag_callback
#    #print("Inside callback")
#    #print("data:",data)
#    #print("flag_callback:",flag_callback)
#    if(flag_callback == 1):
#        flag_callback = 0
#        print("data_inside:",data)
#        return data
#
#def callback_image(data):
#    #Accessing data at around 30FPS
#    #print("Inside callback function")
#    #end_time = time.time()
#    #print("Time:",end_time - start_time)
#    #start_time = time.time()
#    bridge = CvBridge()
#    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
#    return cv_image
#
#
#def subscribe_lidar():
#    rospy.init_node('listener',anonymous=True)
#    depth = rospy.Subscriber('/lidar',Float32,callback_depth)
#    rospy.spin()
#    print("depth:",depth)
#    cnt_detect.depth = depth


def publish_data(box_pose):
    pub = rospy.Publisher('chatter',String,queue_size=10)
    #rospy.init_node('talker',anonymous=True)
    rate = rospy.Rate(100)
    str_to_publish = str(box_pose)
    pub.publish(str_to_publish)
    rate.sleep()

def draw_ids(img,association_id):
    winName = "Tracking"
    cv.namedWindow(winName,cv.WINDOW_NORMAL)
    for (key,value) in (association_id.items()):
        img = cv.putText(img,str(key),(value[0],value[1]),cv.FONT_HERSHEY_SIMPLEX,1.0,(255,255,255),2)
    cv.imshow(winName,img)
    cv.waitKey(1)

if __name__ == "__main__":
    
    track = track_id()  
    #track_kf = tracker_kf()
    while(not (rospy.is_shutdown())):
        start_time = time.time()
        cnt_detect = contour_process()
        det_pose = pose_estimation()
        #Subscribing to the image
        frame = subscribe_image()
        
        winName = "Live feed"
        cv.namedWindow(winName,cv.WINDOW_NORMAL)
        cv.imshow(winName,frame)
        cv.waitKey(1)
        ori_img = np.copy(frame)
        ori_img_1 = np.copy(frame)
        
        box_all,flag = cnt_detect.colour_analyse(frame)        
        #process_track(box_all,ori_img_1)
        if(flag == 1):
            #theta_ls , depth_ls , transx_ls , transy_ls , flag_pose = det_pose.process_pose_1(ori_img,box_all)
            vis_img ,box_pose,tl_ls,br_ls,flag_pose = det_pose.process_pose_1(ori_img,box_all)
            #Publishing the above data on the ROStopic chatter
            out.write(vis_img)
            publish_data(box_pose)

            #Tracking
            track.process_track(ori_img_1,tl_ls,br_ls)
            #Implementing kalman filter for tracking
            #track_kf.process_kalman(association_id)
        end_time = time.time()
        print("Time taken for entire pipeline:",end_time - start_time)
        out.release()

        #print("dict_debug:",dict_debug)
        #if(len(dict_debug) > 100):
        #    f = open("store_dict.pkl","wb")
        #    pickle.dump(dict_debug,f)
        #    f.close()
        #    print("Done writing data!!!")
    #winName = "Live feed"
    #cv.namedWindow(winName,cv.WINDOW_NORMAL)
    #video = "/home/varghese/data_12th_Sept/input.avi"

    #if(video):
    #    cap = cv.VideoCapture(video)
    
    #while(cap.isOpened()):
    
    #for img_path in glob.glob("/home/varghese/challenge_2/brick_train/brick_train_v6/images/*.jpg"):
    #    start_time = time.time()
    #    cnt_detect.clear_all()
    #    det_pose.clear_all()
    #
    #    frame = cv.imread(img_path)
    #    #ret , frame = cap.read()
    #    cv.imshow(winName,frame)
    #    cv.waitKey(0)

    #    ret = True

    #    if not ret:
    #       cap.release()
    #       break
    #
    #    else:
    #        ori_img = np.copy(frame)
    #        box_all,flag = cnt_detect.colour_analyse(frame)
    #        

    #        if(flag == 1):
    #            det_pose.process_pose_1(ori_img,box_all)
    #    end_time = time.time()
    #    print("Number of time in seconds required for entire pipeline:",float(end_time - start_time))
        #inp = input("Waiting for input...")

    #img = cv.imread("/home/varghese/data_25th_sept/video/picture9_033.jpg")
    #ori_img = np.copy(img)
    #box_all , flag = colour_analyse(img)
    #process_pose_1(img,box_all)
