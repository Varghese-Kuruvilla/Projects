#Script to plot the depth values
import pickle
import matplotlib.pyplot as plt
import numpy as np

file_data = open("store_dict.pkl","rb")
dict_data = pickle.load(file_data)

#print("dict_data:",dict_data)

lidar_depth = list(dict_data.keys())
true_depth = list(dict_data.values()) 
    
np_lidar_depth = np.asarray(lidar_depth)
np_true_depth = np.asarray(true_depth) 


diff_depth = np.subtract(np_lidar_depth,np_true_depth)
#error_percent = (diff_depth)/(np_lidar_depth) * 100
#print("Error_percent:",error_percent)
index = np.where(diff_depth == -2850.6248)
diff_depth_modified = np.delete(diff_depth,index)

np_lidar_depth_modified = np.delete(np_lidar_depth,index)
np_true_depth_modified = np.delete(np_true_depth,index)


mean_error = np.mean(diff_depth_modified)
print("Mean_error:",mean_error)
print("Index:",index)
print("diff_depth:",diff_depth)


plt.title("Comparison of lidar depth vs true depth")
plt.ylabel("Depth values")
plt.plot(np_lidar_depth_modified,'r',label= "Lidar depth")
plt.plot(np_true_depth_modified,'b',label = "Computed depth")
plt.plot(diff_depth_modified)
#for a,b in zip(x, y): 
#    pyplot.text(a, b, str(b))
plt.legend()
plt.show()


