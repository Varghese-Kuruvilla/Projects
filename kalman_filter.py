#Script to implement the kalman filter for tracking
import numpy as np
from numpy import dot
from scipy.linalg import inv, block_diag

first = 0
class tracker_kf:
    def __init__(self):
        self.x_state = [] #state which contains [cx,cy,w,h,vx,vy,vw,vh]
        self.dt = 1

        #Process matrix for constant velocity model
        self.F = np.array([[1,0,0,0,self.dt,0,0,0],
                           [0,1,0,0,0,self.dt,0,0],
                           [0,0,1,0,0,0,self.dt,0],
                           [0,0,0,1,0,0,0,self.dt],
                           [0,0,0,0,1,0,0,0],
                           [0,0,0,0,0,1,0,0],
                           [0,0,0,0,0,0,1,0],
                           [0,0,0,0,0,0,0,1]])

        #Measurement matrix 
        self.H = np.array([[1,0,0,0,0,0,0,0],
                           [0,1,0,0,0,0,0,0],
                           [0,0,1,0,0,0,0,0],
                           [0,0,0,1,0,0,0,0]])

        #Covariance matrix
        self.L = 10.0
        self.P = np.diag(self.L*np.ones(8))
        
        #Noise matrix R
        self.R = np.array([[1,0,0,0],
                           [0,1,0,0],
                           [0,0,10,0],
                           [0,0,0,10]])

        self.Q_comp_mat = np.array([[self.dt**4/4., self.dt**3/2.],
                                    [self.dt**3/2., self.dt**2]])
        self.Q = block_diag(self.Q_comp_mat, self.Q_comp_mat, 
                            self.Q_comp_mat, self.Q_comp_mat)

        print("self.Q:",self.Q)

    def init_state(self,j,box):
        global first
        cx = box[0] 
        cy = box[1]
        w = box[2] - box[4]
        h = box[3] - box[5]
        if(first == 0):
            print("Inside first=0 condition")
            first = first + 1
            self.x_state = np.array([cx,cy,w,h,0,0,0,0])
            self.x_state = np.reshape(self.x_state,(8,1))
            print("self.x_state:",self.x_state)
            print("type(self.x_state):",type(self.x_state))
            print("self.x_state.shape:",self.x_state.shape)

        self.z = np.array([cx,cy,w,h])
        self.z = np.reshape(self.z,(4,1))
        updated_state = self.predict_update(self.z)
        return updated_state

    def predict_update(self,z): #z represents the measurement

        x = self.x_state
        #print("self.x_state.shape:",self.x_state.shape)
        #print("x:",x)
        #print("x.shape:",x.shape)
        #Predict
        x = dot(self.F,x)
        self.P = dot(self.F,self.P).dot(self.F.T) + self.Q
        #print("x.shape:",x.shape)
        #print("self.H.shape:",self.H.shape)
        #Update
        S = dot(self.H, self.P).dot(self.H.T) + self.R
        k = dot(self.P, self.H.T).dot(inv(S))
        y = self.z - dot(self.H, x)
        x = x + dot(k,y)

        self.P = self.P - dot(k,self.H).dot(self.P)
        self.x_state = x.astype(int)
        return self.x_state


    def process_kalman(self,association_id):
        for (j,(key,val)) in enumerate((association_id.items())):
            box_state = association_id[key]
            updated_state = self.init_state(j,box_state)
            print("box_state:",box_state)
            print("Updated_state:",updated_state)



