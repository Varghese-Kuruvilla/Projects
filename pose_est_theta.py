#!/usr/bin/env python
# import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2 as cv
import numpy as np 
import glob
from matplotlib import pyplot as plt
from math import *
import time  

#global parameters
success_pose = 0
# winName = "Pose estimation"
# cv.namedWindow(winName, cv.WINDOW_NORMAL)



# def isRotationMatrix(R) :
#     Rt = np.transpose(R)
#     shouldBeIdentity = np.dot(Rt, R)
#     I = np.identity(3, dtype = R.dtype)
#     n = np.linalg.norm(I - shouldBeIdentity)
#     return n < 1e-6

# def getangles(rvecs):
    
#     R=cv.Rodrigues(rvecs)[0]
#     assert(isRotationMatrix(R))
            
#     sy = sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
#     singular = sy < 1e-6
 
#     if  not singular:
#         x = degrees(atan2(R[2,1] , R[2,2]))
#         y = degrees(atan2(-R[2,0], sy))
#         z = degrees(atan2(R[1,0], R[0,0]))
#     else :
#         x = degrees(atan2(-R[1,2], R[1,1]))
#         y = degrees(atan2(-R[2,0], sy))
#         z = 0
#     return x,y,z

# #Converting from quaternion to euler angles
# def quaternion_to_euler(x, y, z, w):


#     t0 = +2.0 * (w * x + y * z)
#     t1 = +1.0 - 2.0 * (x * x + y * y)
#     roll = degrees(atan2(t0, t1))
#     t2 = +2.0 * (w * y - z * x)
#     t2 = +1.0 if t2 > +1.0 else t2
#     t2 = -1.0 if t2 < -1.0 else t2
#     pitch = degrees(asin(t2))
#     t3 = +2.0 * (w * z + x * y)
#     t4 = +1.0 - 2.0 * (y * y + z * z)
#     yaw = degrees(atan2(t3, t4))

    #return yaw, pitch, roll

#loading intrinsics
#mtx = np.array([[700.4,   0. , 628.787], [  0. , 700.4, 372.022],[  0. ,   0. ,   1. ]])
#dist = np.array([[-0.175725, 0.0290343, 0., 0., 1.]]) 
mtx = np.array([[690.08699358 ,  0.,         265.02791994],[  0.,         686.77087112, 243.98903059],[  0.,           0. ,          1.        ]]) #For logitech c615
dist = np.array([[-0.02285312, -0.14753576 , 0.00196857, -0.01649874 , 0.31742216]]) 

def draw(img,corners,imgpts):
    #print("imgpts[1].ravel()",(imgpts[1].ravel()))
    corner = tuple(corners[0].ravel())
    img = cv.line(img,corner,tuple(imgpts[0].ravel()),(255,0,0),5)
    img = cv.line(img,corner,tuple(imgpts[1].ravel()),(0,255,0),5)
    img = cv.line(img,corner,tuple(imgpts[2].ravel()),(0,0,255),5)
    return img

#Loading images in RGB and grayscale
def process_theta(ori_img,tl,tr,br,bl):

    #Finding the slope between points
    avg_theta_long = 0
    avg_theta_short = 0
    slope_1 = (tr[1]-tl[1])/(tr[0]-tl[0])
    theta_1 = degrees(atan(slope_1))
    if(abs(theta_1) >=0 and abs(theta_1)<=90):
        #print("theta_1 condition satisfied")
        avg_theta_long = avg_theta_long + theta_1
    #print("theta_1:",theta_1)

    slope_2 = (br[1]-tr[1])/(br[0]-tr[0])
    theta_2 = degrees(atan(slope_2))
    if(abs(theta_2) >=0 and abs(theta_2)<=90):
        #print("theta_2 condition satisfied")
        avg_theta_short = avg_theta_short + theta_2

    slope_3 = (bl[1] - br[1])/(bl[0] - br[0])
    theta_3 = degrees(atan(slope_3))
    #print("theta_3:",theta_3)
    if(abs(theta_3) >=0 and abs(theta_3)<=90):
        #print("theta_3 condition satisfied")
        avg_theta_long = avg_theta_long + theta_3

    slope_4 = (tl[1] - bl[1])/(tl[0] - bl[0])
    theta_4 = degrees(atan(slope_4))
    #print("theta_4:",theta_4)
    if(abs(theta_4) >=0 and abs(theta_4)<=90):
        #print("condition 4 theta satisfied")
        avg_theta_short = avg_theta_short + theta_4

    avg_theta_long = avg_theta_long /2 
    avg_theta_short = avg_theta_short /2 
    #print("avg_theta_long:",avg_theta_long)
    #print("avg_theta_short:",avg_theta_short)
    
    return [avg_theta_long,avg_theta_short]

def process_depth(ori_img,tl,tr,br,bl):

    fx = 690.0869
    fy = 686.7708
    cx = 265.0279
    cy = 243.9890

    worldlength = 30 #in cms
    pixellength_1 = sqrt((tl[0] - tr[0])**2 + (tl[1] - tr[1])**2)
    pixellength_2 = sqrt((bl[0] - br[0])**2 + (bl[1] - br[1])**2)
    #print("pixellength_1:",pixellength_1)
    #print("pixellength_2:",pixellength_2)
    pixellength = (pixellength_1 + pixellength_2)/2
    depth = (fy * worldlength)/(pixellength)
    trans_x = ((tl[0] - cx) * depth)/(fx)
    trans_y = ((tl[1] - cy) * depth)/(fy)
    #print("Depth in cms:",depth)
    #print("trans_x:",trans_x)
    #print("trans_y:",trans_y)
    #inp = input("waiting for input...")
    return depth,trans_x,trans_y 


def plot_on_img(ori_img,tl,tr,br,bl):
    
    cv.drawMarker(ori_img,(int(tl[0]),int(tl[1])), color=(0,255,0),markerType=cv.MARKER_DIAMOND,markerSize=3,thickness=3)
    cv.drawMarker(ori_img,(int(tr[0]),int(tr[1])), color=(0,255,0),markerType=cv.MARKER_DIAMOND,markerSize=3,thickness=3)
    cv.drawMarker(ori_img,(int(br[0]),int(br[1])), color=(0,255,0),markerType=cv.MARKER_DIAMOND,markerSize=3,thickness=3)
    cv.drawMarker(ori_img,(int(bl[0]),int(bl[1])), color=(0,255,0),markerType=cv.MARKER_DIAMOND,markerSize=3,thickness=3)

def process_pose_1(ori_img,box_all):
    theta_ls = []  #List of the theta values
    depth_ls = []
    transx_ls = []
    transy_ls = []
    print("len(box_all):",len(box_all))
    for i in range(0,len(box_all)):
        if(box_all[i].shape == (4,2)):
            tl = box_all[i][0]
            tr = box_all[i][1]
            br = box_all[i][2]
            bl = box_all[i][3]

            theta = process_theta(ori_img,tl,tr,br,bl) #Returns the theta values of the longer edge and the shorter edge in that order
            theta_ls.append(theta)

            depth_est,trans_x,trans_y = process_depth(ori_img,tl,tr,br,bl)

            depth_ls.append(depth_est)
            transx_ls.append(trans_x)
            transy_ls.append(trans_y)
            plot_on_img(ori_img,tl,tr,br,bl)
    
    #Displaying corners on the image 
    winName = "Visualise corners"
    cv.namedWindow(winName,cv.WINDOW_NORMAL)
    cv.imshow(winName,ori_img)
    cv.waitKey(1)

    print("theta_ls:",theta_ls)
    print("depth_ls:",depth_ls)
    print("transx_ls:",transx_ls)
    print("transy_ls:",transy_ls)
    
    #return theta_ls , depth_ls ,transx_ls,transy_ls,1 #Indicates that pose estimation is successful

#For debugging purposes - if the bounding boxes aren't accurate
if __name__ == '__main__':
    ori_img = cv.imread("/home/varghese/Work/rbccps/MBZIRC/python_scripts/pose_estimation/ground_truth/3.png")
    process_pose(ori_img , 0,0,0,0,0,0)
