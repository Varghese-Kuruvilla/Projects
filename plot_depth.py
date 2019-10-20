#Script to plot the depth values
import pickle
import matplotlib.pyplot as plt

file_data = open("store_dict.pkl","rb")
dict_data = pickle.load(file_data)

print("dict_data:",dict_data)

lidar_depth = list(dict_data.keys())
true_depth = list(dict_data.values())

plt.title("Comparison of lidar depth vs true depth")
plt.ylabel("Depth values")
plt.plot(lidar_depth,'r')
plt.plot(true_depth,'b')
plt.show()


