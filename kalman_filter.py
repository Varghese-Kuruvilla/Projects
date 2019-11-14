#Script to implement the kalman filter for tracking
import numpy as np
from numpy import dot
from scipy.linalg import inv, block_diag

first = 0
class tracker_kf:
    def __init__(self):
        self.x_state = [] #state which contains [cx,cy,w,h,vx,vy,vw,vh]
        self.dt = 1

        self.boxes = []
        self.id = 0 #To store the tracker id
        self.hits = 0 #Total number of frames in which the brick is detected
        self.no_losses = 0 #Total number of frames in which the brick is missed

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
        self.L = 0.1
        self.P = np.diag(self.L*np.ones(8))
        
        #Noise matrix R
        #self.R = np.array([[1,0,0,0],
        #                   [0,1,0,0],
        #                   [0,0,10,0],
        #                   [0,0,0,10]])

        #self.Q_comp_mat = np.array([[self.dt**4/4., self.dt**3/2.],
        #                            [self.dt**3/2., self.dt**2]])

        self.Q_comp_mat = np.array([[5.0,10.0],
                                    [10.0,15.0]])   #These values are hardcoded and should be calculated dynamically
        #print("self.Q_comp_mat:",self.Q_comp_mat)
        self.Q = block_diag(self.Q_comp_mat, self.Q_comp_mat, 
                            self.Q_comp_mat, self.Q_comp_mat)


        #Process covariance
        self.R_scaler = 1.0
        self.R_diag_array = self.R_scaler * np.array([self.L, self.L, self.L, self.L])
        self.R = np.diag(self.R_diag_array)
        print("self.R:",self.R)

        print("self.Q:",self.Q)


    def predict_update(self,z): #z represents the measurement

        x = self.x_state
        print("Current state inside predict_update:",x)
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
        y = z - dot(self.H, x)
        x = x + dot(k,y)

        self.P = self.P - dot(k,self.H).dot(self.P)
        self.x_state = x.astype(int)
        print("State after updation:",self.x_state)
        #return self.x_state

    def predict_only(self):
        x = self.x_state
        #Predict state
        x = dot(self.F,x)
        self.P = dot(self.F,self.P).dot(self.F.T) + self.Q
        self.x_state = x.astype(int)


    def process_kalman(self,association_id):
        for (j,(key,val)) in enumerate((association_id.items())):
            box_state = association_id[key]
            updated_state = self.init_state(j,box_state)
            print("box_state:",box_state)
            print("Updated_state:",updated_state)



