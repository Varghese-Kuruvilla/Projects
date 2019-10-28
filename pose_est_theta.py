#!/usr/bin/env python
# import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2 as cv
import numpy as np 
import glob
from matplotlib import pyplot as plt
from math import *
import time  

#ROS Dependencies
import rospy
from std_msgs.msg import Float32

#global parameters
success_pose = 0
#global_depth_est = 0
# winName = "Pose estimation"
# cv.namedWindow(winName, cv.WINDOW_NORMAL)


#loading intrinsics
#mtx = np.array([[700.4,   0. , 628.787], [  0. , 700.4, 372.022],[  0. ,   0. ,   1. ]])
#dist = np.array([[-0.175725, 0.0290343, 0., 0., 1.]]) 
#mtx = np.array([[690.08699358 ,  0.,         265.02791994],[  0.,         686.77087112, 243.98903059],[  0.,           0. ,          1.        ]]) #For logitech c615
#dist = np.array([[-0.02285312, -0.14753576 , 0.00196857, -0.01649874 , 0.31742216]])


class pose_estimation:

    def __init__(self):
    
        self.tl = 0
        self.tr = 0
        self.br = 0
        self.bl = 0
        self.theta_ls = []  
        self.depth_ls = []
        self.transx_ls = []
        self.transy_ls = []
        self.box_pose = []

    def clear_all(self):


        self.tl = 0
        self.tr = 0
        self.br = 0
        self.bl = 0
        self.theta_ls = []  
        self.depth_ls = []
        self.transx_ls = []
        self.transy_ls = []

    def draw(img,corners,imgpts):
        #print("imgpts[1].ravel()",(imgpts[1].ravel()))
        corner = tuple(corners[0].ravel())
        img = cv.line(img,corner,tuple(imgpts[0].ravel()),(255,0,0),5)
        img = cv.line(img,corner,tuple(imgpts[1].ravel()),(0,255,0),5)
        img = cv.line(img,corner,tuple(imgpts[2].ravel()),(0,0,255),5)
        return img

    #Loading images in RGB and grayscale
    def process_theta(self,ori_img,tl,tr,br,bl):

        #Finding the slope between points
        avg_theta_long = 0
        avg_theta_short = 0
        if(tr[0] != tl[0]):
            slope_1 = (tr[1]-tl[1])/(tr[0]-tl[0])
            theta_1 = degrees(atan(slope_1))
        else:
            theta_1 = 90
        if(abs(theta_1) >=0 and abs(theta_1)<=90):
            #print("theta_1 condition satisfied")
            avg_theta_long = avg_theta_long + theta_1
        #print("theta_1:",theta_1)
        
        if(br[0] != tr[0]):
            slope_2 = (br[1]-tr[1])/(br[0]-tr[0])
            theta_2 = degrees(atan(slope_2))
        else:
            theta_2 = 90
        if(abs(theta_2) >=0 and abs(theta_2)<=90):
            #print("theta_2 condition satisfied")
            avg_theta_short = avg_theta_short + theta_2

        if(bl[0] != br[0]):
            slope_3 = (bl[1] - br[1])/(bl[0] - br[0])
            theta_3 = degrees(atan(slope_3))
        else:
            theta_3 = 90
        #print("theta_3:",theta_3)
        if(abs(theta_3) >=0 and abs(theta_3)<=90):
            #print("theta_3 condition satisfied")
            avg_theta_long = avg_theta_long + theta_3

        if(tl[0] != bl[0]):
            slope_4 = (tl[1] - bl[1])/(tl[0] - bl[0])
            theta_4 = degrees(atan(slope_4))
        else:
            theta_4 = 90
        #print("theta_4:",theta_4)
        if(abs(theta_4) >=0 and abs(theta_4)<=90):
            #print("condition 4 theta satisfied")
            avg_theta_short = avg_theta_short + theta_4

        avg_theta_long = avg_theta_long /2 
        avg_theta_short = avg_theta_short /2 
        #print("avg_theta_long:",avg_theta_long)
        #print("avg_theta_short:",avg_theta_short)
        
        return [avg_theta_long,avg_theta_short]

    
    def det_longer(self,tl,tr,br,bl):
        pts = np.array([tl,tr,br,bl])
        x_sorted = pts[np.argsort(pts[:,0]),:]
    
        #Grabbing the leftmost and rightmost points from the sorted coordinate points
        leftmost = x_sorted[:2,:]
        rightmost = x_sorted[2:,:]

        
        leftmost = leftmost[np.argsort(leftmost[:,1]),:]
        rightmost = rightmost[np.argsort(rightmost[:,1]),:]
        (top_left , bottom_left) = leftmost
        (top_right , bottom_right) = rightmost
        
        return top_left , top_right, bottom_right , bottom_left



        

    def process_depth(self,ori_img,tl,tr,br,bl):

        tl , tr, br , bl = self.det_longer(tl,tr,br,bl) #Function to find the points in the correct order
        self.tl = tl
        self.tr = tr
        self.br = br
        self.bl = bl

        fx = 617.0869
        fy = 617.7708
        #fx = 530
        #fy = 530
        cx = 332.0279
        cy = 240.9890

        worldlength = 30 #in cms
        pixellength_1 = sqrt((tl[0] - tr[0])**2 + (tl[1] - tr[1])**2)
        #print("tl[0]:",tl[0])
        #print("tl[1]:",tl[1])
        #print("tr[0]:",tr[0])
        #print("tr[1]:",tr[1])
        pixellength_2 = sqrt((bl[0] - br[0])**2 + (bl[1] - br[1])**2)
        pixellength = (pixellength_1 + pixellength_2)/2
        #depth = (fy * worldlength)/(pixellength)
        depth = subs_depth()
        print("depth inside pose_est:",depth.data)
        trans_x = ((tl[0] - cx) * depth.data)/(fx)
        trans_y = ((tl[1] - cy) * depth.data)/(fy)
        #Translation in x and y w.r.t the centre of the brick
        trans_x = trans_x + 15
        trans_y = trans_y + 10
        return depth.data,trans_x,trans_y 


    def plot_on_img(self,ori_img,tl,tr,br,bl):
        
        cv.drawMarker(ori_img,(int(tl[0]),int(tl[1])), color=(0,255,0),markerType=cv.MARKER_DIAMOND,markerSize=3,thickness=3)
        cv.drawMarker(ori_img,(int(tr[0]),int(tr[1])), color=(0,255,0),markerType=cv.MARKER_DIAMOND,markerSize=3,thickness=3)
        cv.drawMarker(ori_img,(int(br[0]),int(br[1])), color=(0,255,0),markerType=cv.MARKER_DIAMOND,markerSize=3,thickness=3)
        cv.drawMarker(ori_img,(int(bl[0]),int(bl[1])), color=(0,255,0),markerType=cv.MARKER_DIAMOND,markerSize=3,thickness=3)

    def process_pose_1(self,ori_img,box_all):
        global global_depth_est 
        global_depth_est = 0
        tl_ls = []
        br_ls = []
        #print("len(box_all):",len(box_all))
        for i in range(0,len(box_all)):
            if(box_all[i].shape == (4,2)):
                self.tl = box_all[i][0]
                self.tr = box_all[i][1]
                self.br = box_all[i][2]
                self.bl = box_all[i][3]

                tl_ls.append(self.tl)
                br_ls.append(self.br)
                theta = self.process_theta(ori_img,self.tl,self.tr,self.br,self.bl) #Returns the theta values of the longer edge and the shorter edge in that order
                self.theta_ls.append(theta)

                depth_est,trans_x,trans_y = self.process_depth(ori_img,self.tl,self.tr,self.br,self.bl)
                self.box_pose.append([theta , depth_est , trans_x , trans_y])
                self.plot_on_img(ori_img,self.tl,self.tr,self.br,self.bl)
        
        #Displaying corners on the image 
        winName = "Visualise corners"
        cv.namedWindow(winName,cv.WINDOW_NORMAL)
        cv.imshow(winName,ori_img)
        cv.waitKey(1)
        
        #return self.box_pose , global_depth_est, 1
        return ori_img,self.box_pose,tl_ls,br_ls,1


def subs_depth():
    depth = rospy.wait_for_message('/lidar',Float32)
    return depth

#For debugging purposes - if the bounding boxes aren't accurate
if __name__ == '__main__':
    ori_img = cv.imread("/home/varghese/Work/rbccps/MBZIRC/python_scripts/pose_estimation/ground_truth/3.png")
    process_pose(ori_img , 0,0,0,0,0,0)
