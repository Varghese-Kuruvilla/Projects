#Converting images into YUV colourspace
import sys
sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
import cv2 as cv
import numpy as np
import glob
import matplotlib.pyplot as plt
from statistics import mean
from pose_est_theta import process_pose_1
import time

#Global variables
area_anomaly = [] #List for detection of area anomalies
fx = 690.08
fy = 686.77
cx = 265.02
cy = 243.98

class contour_process:

    def validate_cnt(cnt):
        global fy
        depth = 400
        threshold = 20
        perimeter_cnt = cv.arcLength(cnt,True)

        pixel_length = (fy * 30)/(depth)
        pixel_breadth = (fy * 20)/(depth)

        perimeter_depth = 2 * (pixel_length + pixel_breadth)

        print("perimeter_cnt:",perimeter_cnt)
        print("perimeter_depth:",perimeter_depth)
        
        if(abs(perimeter_cnt - perimeter_depth)<=20):
            return 1 #Indicates that the contour should be considered for further processing
        else:
            return 0
        

    def find_slope(self,seg,img,hist,v_channel,mask,mask_erode):
        box_all = [] #List of np arrays which contains box coordinates
        #seg is the segmented image
        #img is the original image on which we can draw the contours
                                  #hist is only for debugging purposes
        # draw_cnt = segs
        img_approx = np.copy(img)
        img_rect = np.copy(img)
        img_cnt = np.copy(img)


        #Finding out the contours
        self.cnt , hierarchy = cv.findContours(seg , cv.RETR_TREE , cv.CHAIN_APPROX_SIMPLE)
        #print("hierarchy:",hierarchy)
        #print("len(cnt):",len(cnt))



        #Drawing contours on the image    
        cv.drawContours(img_cnt , self.cnt , -1 , (0,255,0) , 3)
        #winName_cnt = "Drawing contours"
        #cv.namedWindow(winName_cnt,cv.WINDOW_NORMAL)
        #cv.imshow(winName_cnt,img_cnt)
        #cv.waitKey(0)


        #So if the hierarchy of [i][3] is zero we fit a rotated rect to that particular contour
        #winName_rect = "Rotated rectangle"
        #cv.namedWindow(winName_rect,cv.WINDOW_NORMAL)

        for i in range(0, len(hierarchy[0])):
            if(hierarchy[0][i][3] == 0):
                #Validating the contours using area considerations   
                flag_valid_cnt = validate_cnt(self.cnt[i])
                if(flag_valid_cnt == 1):
                    rect = cv.minAreaRect(self.cnt[i])
                    #print("rect:",rect)
                    box = cv.boxPoints(rect)
                    box = np.int0(box)
                    self.box_all.append(box)

                #cv.drawContours(img_rect,[box],0,(0,0,255),2)
                #cv.imshow(winName_rect,img_rect)
                #cv.waitKey(0)

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
        self.img_yuv = cv.cvtColor(img , cv.COLOR_BGR2YUV)
        v_channel = img_yuv[:,:,2]

        winName_1 = "V channel"
        cv.namedWindow(winName_1 , cv.WINDOW_NORMAL)
        cv.imshow(winName_1 , v_channel)
        cv.waitKey(0)

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
        
        self.hist = cv.calcHist([v_channel],[0],None,[256],[0,256])
        
        plt.plot(hist)
        plt.xlim([0,256])
        plt.show()


        
        #Calculating the variance
        #std_v = np.std(v_channel)
        #print("std_v:",std_v)
        std_v = 33.0   #This should be calculated dynamically

        mean_v = np.mean(v_channel)
        #print("mean_v:",mean_v)
        #inp = input("waiting for input...")

        mean = np.where(hist == np.amax(hist)) #Mean is considered as the maximum point in the histogram
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
        
        #Apply 2 masks for the image
        #Shows a peak at 150: so consider range of colours from 100 to 200 i.e mean - (30% of mean) to mean + (30% of mean)
        #mask_1 = cv.inRange(v_channel,lower_thresh_1,upper_thresh_1)
        #mask_2 = cv.inRange(v_channel,lower_thresh_2,upper_thresh_2)
        #mask = mask_1 + mask_2

        self.mask = cv.inRange(v_channel , lower_thresh , upper_thresh)

        #winName_3 = "Mask_Image"
        #cv.namedWindow(winName_3,cv.WINDOW_NORMAL)
        #cv.imshow(winName_3,mask)
        #key = cv.waitKey(0)
        #if(key & 0xFF == ord('q')):
        #    cv.destroyAllWindows()


        #The mask contains some holes, so we can try to erode part of the mask
        self.kernel_erode = np.ones((3,3),np.uint8)
        self.mask_erode = cv.erode(mask,kernel_erode,iterations = 1)
        

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
     
        box_all , flag = find_slope(self.mask_erode,img,self.hist,self.v_channel,self.mask,self.mask_erode) #hist and v_channel are for debugging

        if(flag == 1):
            return box_all , flag
        else:
            return 0,0

        #return 1,1


    #def process_colour(img):
    #    approx_cnt , flag = colour_analyse(img)
    #    return approx_cnt , flag #If flag = 0 then it indicates an error

if __name__ == "__main__":
    
    cnt_detect = contour_process() 
    winName = "Live feed"
    cv.namedWindow(winName,cv.WINDOW_NORMAL)
    #video = "/home/varghese/darknet/test_videos/output_18_sept.avi"

    #if(video):
    #    cap = cv.VideoCapture(video)
    
    #while(cap.isOpened()):

    for img_path in glob.glob("/home/varghese/challenge_2/brick_train/brick_train_v6/images/*.jpg"):
        frame = cv.imread(img_path)
        #approx_cnt , flag = colour_analyse(frame)
        #ret , frame = cap.read()
        cv.imshow(winName,frame)
        cv.waitKey(0)
        ret = True
        if not ret:
           print("done processing")
           cap.release()
           break
    
        else:
            ori_img = np.copy(frame)
            box_all,flag = cnt_detect.colour_analyse(frame)
            
            if(flag == 1):
                process_pose_1(ori_img,box_all)

        

    #img = cv.imread("/home/varghese/data_25th_sept/video/picture9_033.jpg")
    #ori_img = np.copy(img)
    #box_all , flag = colour_analyse(img)
    #process_pose_1(img,box_all)
