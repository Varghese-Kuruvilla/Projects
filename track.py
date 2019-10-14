#Script to track objects using dlib tracker
import dlib
import cv2 as cv

#Global variables 
label_count = 0
class track_brick:
    def __init__(self):
        label = 0
        labels = [] #List of labels
        trackers = [] #List of trackers 


    def first_update(self,greatest_ls,least_ls,frame):
        print("Inside first update:")
        global label_count
        for i in range(0,len(greatest_ls)):
            startX,startY = least_ls[i]
            endX,endY = greatest_ls[i]
            print("startX,startY,endX,endY:",startX,startY,endX,endY)

            t = dlib.correlation_tracker()
            rect = dlib.rectangle(startX,startY,endX,endY)
            t.start_track(frame,rect)
            
            self.labels.append(self.label)
            label = label + 1
            self.trackers.append(t)

            #Drawing the rectangle on the image
            cv.rectangle(frame,(startX,startY),(endX,endY),(0,255,0),2)
            cv.putText(frame,label,(startX,startY-15),cv.FONT_HERSHEY_SIMPLEX,0.45,(0,255,0),2)

        label_count = label

    def sbs_update(self,frame):
        print("Inside subsequent update:")
        for (t,l) in zip(self.trackers,self.labels):
            #Update the position of the objects
            t.update(frame)

            pos = t.get_position()
            startX = int(pos.left())
            startY = int(pos.top())
            endX = int(pos.right())
            endY = int(pos.bottom())


            cv.rectangle(frame,(startX,startY),(endX,endY),(0,255,0),2)
            cv.putText(frame,label,(startX,startY-15),cv.FONT_HERSHEY_SIMPLEX,0.45,(0,255,0),2)
    
            
            
            









def process_track(box_all,frame):
    global label_count 

    greatest_x = 0
    greatest_y = 0
    greatest_index = 1000
    greatest_ls = []

    least_x = -100
    least_index = 1000
    least_ls = []

    if(box_all!= 0):
        print("len(box_all):",len(box_all))
        print("box_all:",box_all)
        for (i,box) in enumerate(box_all):
            greatest_x = box[0][0] #Initially make greatest_x as the x coordinate of the first point
            greatest_y = box[0][1]
            greatest_index = 0
            least_x = box[0][0] 
            least_y = box[0][1]
            least_index = 0
            for j in range(0,4):
                if(greatest_x < box[j][0] and greatest_y < box[j][1]):
                    greatest_x = box[j][0]
                    greatest_y = box[j][1]
                    greatest_index = j
                
                if(least_x > box[j][0] and least_y > box[j][1]):
                    least_x = box[j][0]
                    least_y = box[j][1]
                    least_index = j
                
            least_ls.append([box[least_index][0],box[least_index][1]])
            greatest_ls.append([box[greatest_index][0],box[greatest_index][1]])

        print("greatest_ls:",greatest_ls)
        print("least_ls:",least_ls)
        
        track_obj = track_brick()
        if(len(box_all) != label_count):
            track_obj.first_update(greatest_ls,least_ls,frame)
        else:
            track_obj.sbs_update(frame)
