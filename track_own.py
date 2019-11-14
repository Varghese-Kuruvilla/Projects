#Script to implement tracking. Proposed method is to associate ids with each detected brick
import numpy as np
from math import sqrt
from sklearn.utils.linear_assignment_ import linear_assignment
from kalman_filter import tracker_kf
from collections import deque
import cv2 as cv

#Global variables
max_age = 4
min_hits = 1

class track_id:
    def __init__(self):
        self.detections = [] #List of detections    
        self.trackers = [] #List of trackers
        self.unmatched_trackers = [] #List which holds info regarding unmatched trackers
        self.unmatched_detections = [] #List holding info regarding unmatched detections
        self.matches = []
        self.track_id_list = deque(['0','1','2','3','4','5'])
        
    def update(self):
        cost_mat = np.zeros((len(self.trackers),len(self.detections)),dtype = np.float32)
        for (t,trk) in enumerate(self.trackers):
            for (d,det) in enumerate(self.detections):
                #Cost matrix
                track_centre = [trk.boxes[0],trk.boxes[1]]
                detection_centre = [det[0],det[1]]
                cost_mat[t][d] = self.dist(track_centre,detection_centre)


        matched_idx = linear_assignment(cost_mat)
        print("Matched_idx:",matched_idx)
        self.unmatched_trackers = []
        for (t,trk) in enumerate(self.trackers):
            if(t not in matched_idx[:,0]):
                print("Inside self.unmatched_trackers")
                print("t:",t)
                self.unmatched_trackers.append(t)
                print("self.unmatched_trackers:",self.unmatched_trackers)
                print("len(self.unmatched_trackers):",len(self.unmatched_trackers))

        self.unmatched_detections = []
        for (d,det) in enumerate(self.detections):
            print("d,det:",d,det)
            if(d not in matched_idx[:,1]):
                self.unmatched_detections.append(d)

        self.matches = []

        for m in matched_idx:
            self.matches.append(m.reshape(1,2))

        if(len(self.matches) == 0):
            self.matches = np.empty((0,2),dtype=int)
        else:
            self.matches = np.concatenate(self.matches,axis=0)

        #return self.matches , self.unmatched_detections, self.unmatched_trackers
        #print("Matched_idx:",matched_idx)
        #print("self.matches:",self.matches)
        #print("self.unmatched_detections",self.unmatched_detections)
        #print("self.unmatched_trackers",self.unmatched_trackers)

               
         
    def dist(self,track_centre,detections_centre):
        #print("track_centre:",track_centre)
        #print("detections_centre:",detections_centre)
        #Calculate the euclidean distance between the points value and match
        dist = sqrt((track_centre[0]-detections_centre[0])**2 + (track_centre[1] - detections_centre[1])**2)
        return dist

    def pipeline(self,img,tl_ls,br_ls):
        #Clear the list of detections
        self.detections = []
        x_box = []

        if(len(self.trackers)>0):
            for trk in self.trackers:
                x_box.append(trk.boxes)
        #Computing centroids
        for i in range(0,len(tl_ls)):
            cent_x = (tl_ls[i][0] + br_ls[i][0]) // 2
            cent_y = (tl_ls[i][1] + br_ls[i][1]) // 2
            width = br_ls[i][0] - tl_ls[i][0] 
            height = br_ls[i][1] - tl_ls[i][1]

            temp_centroid = np.array([cent_x,cent_y,width,height])
            self.detections.append(temp_centroid)
            #print("self.detections:",self.detections)

        self.update() #Carries out matching using the hungarian algorithm


        if(self.matches.size > 0):
            for trk_idx, det_idx in self.matches:
                z = self.detections[det_idx]
                z = np.expand_dims(z,axis=0).T
                print("Current detection:",z)
                trk = self.trackers[trk_idx]
                print("Current tracker state:",trk.boxes)
                trk.predict_update(z)
                xx = trk.x_state.T[0].tolist()
                xx = [xx[0],xx[1],xx[2],xx[3]]
                x_box[trk_idx] = xx
                trk.boxes = xx
                print("Updated tracker state:",trk.boxes)
                trk.hits = trk.hits + 1
                trk.no_losses = 0

        if(len(self.unmatched_detections) >0):
            print("Inside unmatched_detections")
            for idx in self.unmatched_detections:
                z = self.detections[idx]
                z = np.expand_dims(z,axis=0).T
                x = np.array([[z[0], z[1], z[2], z[3], 0, 0, 0, 0]]).T

                trk = tracker_kf()
                trk.x_state = x
                trk.predict_only()

                xx = trk.x_state
                xx = xx.T[0].tolist()
                xx = [xx[0], xx[1], xx[2], xx[3]]
               
                print("x:",x)
                print("xx:",xx)

                trk.boxes = xx
                trk.hits = trk.hits + 1
                trk.id = self.track_id_list.popleft()
                self.trackers.append(trk) #Adding object to the list self.tracker
                x_box.append(xx) #x_box

            print("self.unmatched_trackers ver2:",self.unmatched_trackers)

        if(len(self.unmatched_trackers) > 0):
            print("Inside unmatched_trackers")
            for idx in self.unmatched_trackers:
                tmp_trk = self.trackers[idx]
                tmp_trk.no_losses = tmp_trk.no_losses + 1
                print("tmp_trk.no_losses:",tmp_trk.no_losses)
                print("self.trackers[idx].no_losses:",self.trackers[idx].no_losses)
                tmp_trk.predict_only()
                xx = tmp_trk.x_state
                xx = xx.T[0].tolist()
                xx = [xx[0], xx[1], xx[2], xx[3]]
                tmp_trk.boxes = xx
                x_box[idx] = xx




        print("Trackers list:",self.trackers)

        good_tracker_list = []
        for trk in self.trackers:
            if((trk.hits >= 1) and (trk.no_losses <= 4)):
                good_tracker_list.append(trk)
                box_draw = trk.boxes
                self.draw_ids(img,box_draw,trk.id)
            #print("box_draw:",box_draw)
            #print("trk.id:",trk.id)


        #Keeping track of the deleted tracks
        #deleted_tracks = filter(lambda x:x.no_losses>max_age,self.trackers)
        deleted_tracks = [x for x in self.trackers if x.no_losses>max_age]
        print("len(deleted_tracks):",len(deleted_tracks))

        #print("len(list(deleted_tracks)):",len(list(deleted_tracks)))

        for trk in deleted_tracks:
            self.track_id_list.append(trk.id)
            print("self.track_id_list:",self.track_id_list)
            #inp = input("Waiting for input")

        self.trackers = [x for x in self.trackers if x.no_losses <= max_age]
        
        print("self.trackers after deletion:",self.trackers)
                    

    def process_track(self,img,tl_ls,br_ls):
        print("tl_ls:",tl_ls)
        print("br_ls:",br_ls)
        self.pipeline(img,tl_ls,br_ls)


    def draw_ids(self,img,box,trk_id):
        winName = "Tracking"
        cv.namedWindow(winName,cv.WINDOW_NORMAL)
        img = cv.putText(img,str(trk_id),(box[0],box[1]),cv.FONT_HERSHEY_SIMPLEX,1.0,(255,255,255),2)
        cv.imshow(winName,img)
        cv.waitKey(1)


