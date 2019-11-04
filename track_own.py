#Script to implement tracking. Proposed method is to associate ids with each detected brick
import numpy as np
from math import sqrt
from sklearn.utils.linear_assignment_ import linear_assignment
from kalman_filter import tracker_kf
from collections import deque

class track_id:
    def __init__(self):
        self.detections = [] #List of detections    
        self.trackers = [] #List of trackers
        self.unmatched_trackers = [] #List which holds info regarding unmatched trackers
        self.unmatched_detections = [] #List holding info regarding unmatched detections
        self.matches = []
        self.track_id_list = deque(['0','1','2','3','4','5'])

    #def update(self):
    #    #Compare the number of elements in centroids and the associate_id dict
    #    print("self.association_id:",self.association_id)
    #    flag_match = 0
    #    if(len(self.centroids) > len(self.association_id)):
    #        add_new = len(self.centroids) - len(self.association_id)
    #        rem = 0
    #    
    #    elif(len(self.centroids) < len(self.association_id)):
    #        rem = len(self.association_id) - len(self.centroids)
    #        add_new = 0

    #    else:
    #        add_new = 0
    #        rem = 0
    #    

    #    print("add_new:",add_new)
    #    print("rem:",rem)
    #    self.match_ids()

    #    if(add_new > 0):
    #        #Check the centroids that are in self.centroids and aren't in self.associate_id
    #        for i in range(0,len(self.centroids)):
    #            match = self.centroids[i]
    #            for (key,value) in (self.association_id.items()):
    #                if(match[0] == value[0] and match[1] == value[1]):                        
    #                    flag_match = 1

    #            if(flag_match == 0):
    #                print("Inside flag_match:",flag_match)
    #                self.association_id[self.id_no] = match 
    #                self.id_no = self.id_no + 1 
    #            flag_match = 0

    #    print("self.association_id:",self.association_id)
        

    def update(self):
        cost_mat = np.zeros((len(self.trackers),len(self.detections)),dtype = np.float32)
        for (t,trk) in enumerate(self.trackers):
            for (d,det) in enumerate(self.detections):
                #Cost matrix
                track_centre = [trk.boxes[0],trk.boxes[1]]
                detection_centre = [det[0],det[1]]
                cost_mat[t][d] = self.dist(track_centre,detection_centre)


        matched_idx = linear_assignment(cost_mat)

        for (t,trk) in enumerate(self.trackers):
            if(t not in matched_idx[:,0]):
                self.unmatched_trackers.append(t)

        for (d,det) in enumerate(self.detections):
            print("d,det:",d,det)
            if(d not in matched_idx[:,1]):
                print("Inside unmatched detections")
                self.unmatched_detections.append(d)

        for m in matched_idx:
            self.matches.append(m.reshape(1,2))

        if(len(self.matches) == 0):
            self.matches = np.empty((0,2),dtype=int)
        else:
            self.matches = np.concatenate(self.matches,axis=0)

        #return self.matches , self.unmatched_detections, self.unmatched_trackers
        print("Matched_idx:",matched_idx)
        print("self.matches:",self.matches)
        print("self.unmatched_detections",self.unmatched_detections)
        print("self.unmatched_trackers",self.unmatched_trackers)
        #self.unmatched_detections = np.array(self.unmatched_detections)
        #self.unmatched_trackers = np.array(self.unmatched_trackers)

               
         
    def dist(self,track_centre,detections_centre):
        print("track_centre:",track_centre)
        print("detections_centre:",detections_centre)
        #Calculate the euclidean distance between the points value and match
        dist = sqrt((track_centre[0]-detections_centre[0])**2 + (track_centre[1] - detections_centre[1])**2)
        return dist

    def pipeline(self,tl_ls,br_ls):
        #Clear the list of detections
        self.detections = []
        x_box = []
        #Computing centroids
        for i in range(0,len(tl_ls)):
            cent_x = (tl_ls[i][0] + br_ls[i][0]) // 2
            cent_y = (tl_ls[i][1] + br_ls[i][1]) // 2
            width = tl_ls[i][0] - br_ls[i][0]
            height = tl_ls[i][1] - br_ls[i][1]

            temp_centroid = np.array([cent_x,cent_y,width,height])
            self.detections.append(temp_centroid)
            print("self.detections:",self.detections)

        self.update()


        if(self.matches.size > 0):
            for trk_idx, det_idx in self.matches:
                z = self.detections[det_idx]
                z = np.expand_dims(z,axis=0).T
                trk = self.trackers[trk_idx]
                trk.predict_update(z)
                xx = trk.x_state.T[0].tolist()
                xx = [xx[0],xx[1],xx[2],xx[3]]
                x_box[trk_idx] = xx
                trk.boxes = xx
                trk.hits = trk.hits + 1
                trk.no_losses = 0

        if(len(self.unmatched_detections) >0):
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
                trk.id = self.track_id_list.popleft()
                self.trackers.append(trk) #Adding object to the list self.tracker
                x_box.append(xx) #x_box 


        print("Trackers list:",self.trackers)

        good_tracker_list = []
        for trk in self.trackers:
            if((trk.hits >= 1) and (trk.no_losses <= 4)):
                good_tracker_list.append(trk)
                box_draw = trk.boxes
                print("box_draw:",box_draw)


            
           # print("tl_ls[i][0]:",tl_ls[i][0])
           # print("br_ls[i][0]:",br_ls[i][0])
                    

    def process_track(self,tl_ls,br_ls):
        print("tl_ls:",tl_ls)
        print("br_ls:",br_ls)
        self.pipeline(tl_ls,br_ls)




