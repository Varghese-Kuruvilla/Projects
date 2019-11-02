#Script to implement tracking. Proposed method is to associate ids with each detected brick
import numpy as np
from math import sqrt
from sklearn.utils.linear_assignment_ import linear_assignment

class track_id:
    def __init__(self):
        self.association_id = {} #Dictionary to store centroids with the corresponding ids. Key corresponds to the frame id and value corresponds to the (x,y) coordinates of the centroids
        self.centroids = [] #List of centroids where each element in the list is a numpy array of (x,y) coordinates    
        self.id_no = 0

    def update(self):
        #Compare the number of elements in centroids and the associate_id dict
        print("self.association_id:",self.association_id)
        flag_match = 0
        if(len(self.centroids) > len(self.association_id)):
            add_new = len(self.centroids) - len(self.association_id)
            rem = 0
        
        elif(len(self.centroids) < len(self.association_id)):
            rem = len(self.association_id) - len(self.centroids)
            add_new = 0

        else:
            add_new = 0
            rem = 0
        

        print("add_new:",add_new)
        print("rem:",rem)
        self.match_ids()

        if(add_new > 0):
            #Check the centroids that are in self.centroids and aren't in self.associate_id
            for i in range(0,len(self.centroids)):
                match = self.centroids[i]
                for (key,value) in (self.association_id.items()):
                    if(match[0] == value[0] and match[1] == value[1]):                        
                        flag_match = 1

                if(flag_match == 0):
                    print("Inside flag_match:",flag_match)
                    self.association_id[self.id_no] = match 
                    self.id_no = self.id_no + 1 
                flag_match = 0

        print("self.association_id:",self.association_id)
        


    def match_ids(self):
        #cost = np.zeros((3,3))
        cost = np.zeros((len(self.centroids),len(self.association_id)))
        key_mat = np.zeros(len(self.association_id))
        for i in range(0,len(self.centroids)):
            match = self.centroids[i]
            print("match:",match)
            for (j,(key,value)) in enumerate(self.association_id.items()):
                if(i == 0):
                    key_mat[j] = key
                temp_dist = self.dist(value,match)
                print("temp_dist:",temp_dist)
                cost[i][j] = temp_dist
       
        print("cost:",cost)
        res = linear_assignment(cost)
        print("res:",res)
        for i in range(0,len(res)):
            self.association_id[key_mat[i]]=self.centroids[i]
            print("key_mat[i]:",key_mat[i])
            #print("self.association_id[key_mat[i]]:",self.association_id[key_mat[i]])
            print("self.centroids[i]:",self.centroids[i])
               
         
    def dist(self,value,match):
        #Calculate the euclidean distance between the points value and match
        dist = sqrt((value[0]-match[0])**2 + (value[1] - match[1])**2)
        return dist

    def compute_centroids(self,tl_ls,br_ls):
        #Computing centroids
        for i in range(0,len(tl_ls)):
            cent_x = (tl_ls[i][0] + br_ls[i][0]) // 2
            cent_y = (tl_ls[i][1] + br_ls[i][1]) // 2
            temp_centroid = np.array([cent_x,cent_y,tl_ls[i][0],tl_ls[i][1],br_ls[i][0],br_ls[i][1]])
            self.centroids.append(temp_centroid)
            print("self.centroids:",self.centroids)

        self.update()
            
           # print("tl_ls[i][0]:",tl_ls[i][0])
           # print("br_ls[i][0]:",br_ls[i][0])
                    

    def process_track(self,tl_ls,br_ls):
        self.centroids = []
        print("tl_ls:",tl_ls)
        print("br_ls:",br_ls)
        self.compute_centroids(tl_ls,br_ls)
        return self.association_id




