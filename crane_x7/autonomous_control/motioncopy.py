#!/usr/bin/env python
# -*- coding: utf-8 -*-
import datetime
import numpy as np
# import chainer.serializers
# import matplotlib.pylab as plt
# from chainer import Chain, Variable, cuda, optimizer, optimizers, serializers
# import chainer.functions as F
# import chainer.links as L
# from mpl_toolkits.mplot3d.axes3d import Axes3D
# from statistics import mean, median,variance,stdev
# import cv2
# import matplotlib.pyplot as plt
# import math


#from __future__ import print_function
import socket
import select


in_size = 18
out_size = 18
# filepath = "../../DataMake/motion_copy/"
filename = './crane_x7/build/slave1.csv'

# モデルの定義
#train_data = np.loadtxt("../DataMake/TrainData10s/train20.csv",delimiter=",", dtype = np.float32)
train_data = np.loadtxt(
	filename,
	delimiter=',',
	dtype=np.float32,
	skiprows=1,
)
print(train_data.shape)
train_m = []
for i in range(len(train_data)-1):
  train_m.append(train_data[i+1][in_size:in_size+out_size])
width = np.array(train_data[9][1], dtype="float32").reshape((1,))

print(train_m[0])

# def ShowTrajectory(predict, SlaveData):
# 	predict = np.array(predict,dtype="float32")

# 	#SlaveData = Normalization.AntiNormDATA(SlaveData, LSTM_OutSize)
# 	LenPredict = len(predict)
# 	LenSlaveData = len(SlaveData)
# 	if LenPredict > LenSlaveData:
# 		predict = np.delete(predict, slice(len(SlaveData), None), axis=0)
# 	elif LenSlaveData > LenPredict:
# 		SlaveData = np.delete(SlaveData, slice(len(predict), None), axis=0)
# 	SlaveData = np.array(SlaveData,dtype="float32")

#   #動き出しをグラフにするとスケールが違いすぎてよく見えないからカット
# 	#SlaveData = np.delete(SlaveData, slice(None, 50), axis=0)
# 	#predict = np.delete(predict, slice(None, 50), axis=0)

# 	# グラフを描写
# 	fig=plt.figure()
# 	# パラメータ

# 	plt.subplot(3,3,1)
# 	plt.title("Angle Joint 0")
# 	plt.plot(range(len(predict)), SlaveData.T[0], color="blue", label="SlaveData")
# 	plt.plot(range(len(predict)), predict.T[0], color="red", label="predict")
# 	plt.legend()

# 	plt.subplot(3,3,2)
# 	plt.title("Angle Joint 1")
# 	plt.plot(range(len(predict)), SlaveData.T[1], color="blue", label="SlaveData")
# 	plt.plot(range(len(predict)), predict.T[1], color="red", label="predict")
# 	plt.legend()

# 	plt.subplot(3,3,3)
# 	plt.title("Angle Joint 2")
# 	plt.plot(range(len(predict)), SlaveData.T[2], color="blue", label="SlaveData")
# 	plt.plot(range(len(predict)), predict.T[2], color="red", label="predict")
# 	plt.legend()

# 	plt.subplot(3,3,4)
# 	plt.title("AngularVelocity Joint 0")
# 	plt.plot(range(len(predict)), SlaveData.T[3], color="blue", label="SlaveData")
# 	plt.plot(range(len(predict)), predict.T[3], color="red", label="predict")
# 	plt.legend()

# 	plt.subplot(3,3,5)
# 	plt.title("AngularVelocity Joint 1")
# 	plt.plot(range(len(predict)), SlaveData.T[4], color="blue", label="SlaveData")
# 	plt.plot(range(len(predict)), predict.T[4], color="red", label="predict")
# 	plt.legend()

# 	plt.subplot(3,3,6)
# 	plt.title("AngularVelocity Joint 2")
# 	plt.plot(range(len(predict)), SlaveData.T[5], color="blue", label="SlaveData")
# 	plt.plot(range(len(predict)), predict.T[5], color="red", label="predict")
# 	plt.legend()

# 	plt.title("Torque")

# 	plt.subplot(3,3,7)
# 	plt.title("Torque Joint 0")
# 	plt.plot(range(len(predict)), SlaveData.T[6], color="blue", label="SlaveData")
# 	plt.plot(range(len(predict)), predict.T[6], color="red", label="predict")
# 	plt.legend()

# 	plt.subplot(3,3,8)
# 	plt.title("Torque Joint 1")
# 	plt.plot(range(len(predict)), SlaveData.T[7], color="blue", label="SlaveData")
# 	plt.plot(range(len(predict)), predict.T[7], color="red", label="predict")
# 	plt.legend()

# 	plt.subplot(3,3,9)
# 	plt.title("Torque Joint 2")
# 	plt.plot(range(len(predict)), SlaveData.T[8], color="blue", label="SlaveData")
# 	plt.plot(range(len(predict)), predict.T[8], color="red", label="predict")
# 	plt.legend()

# 	plt.show()



def main():
	print("\nソケット間通信を行ってください。")
	host = '127.0.0.1'
	port = 10051
	backlog = 10
	bufsize = 4096
	st = datetime.datetime.now()
	i = 0
	j = 0
	server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	readfds = set([server_sock])
	predict = []
	try:
		server_sock.bind((host, port))
		server_sock.listen(backlog)
	
		while i == 0:
		
			j += 1
			rready, wready, xready = select.select(readfds, [], [])
			for sock in rready:
				if sock is server_sock:
					conn, address = server_sock.accept()
					readfds.add(conn)
				else:
					msg = sock.recv(bufsize)
					if len(msg) == 0:
						sock.close()
						readfds.remove(sock)
					else:
						if msg == "**":
							print("=>"+msg)
							i = 1
						else:
							dd = [] 
							#print("--")
							smsg = msg.split()
							for m in smsg:
								dd.append(float(m))
							input_data_model = np.array(dd, dtype="float32")
							print("///////  time : {:.3f}  ///////\n".format(input_data_model[in_size]))
							input_data_model = np.delete(input_data_model,in_size,0)
							predict.append(input_data_model)	
							input_data_model = np.r_[ input_data_model]	
							print(input_data_model.shape)
	
							print("RECV_position\t\t:\t{:.4f}\t\t{:.4f}\t\t{:.4f}".format(input_data_model[0],input_data_model[1],input_data_model[2]))
							print("RECV_velocity\t:\t{:.4f}\t\t{:.4f}\t\t{:.4f}".format(input_data_model[3],input_data_model[4],input_data_model[5]))
							print("RECV_force\t\t:\t{:.5}\t\t{:.5}\t\t{:.5}".format(input_data_model[6],input_data_model[7],input_data_model[8]))
							print("\n")
	
							print("RNN_position\t\t:\t{:.4f}\t\t{:.4f}\t\t{:.4f}".format(train_m[j-1][0],train_m[j-1][1],train_m[j-1][2]))
							print("RNN_velocity\t:\t{:.4f}\t\t{:.4f}\t\t{:.4f}".format(train_m[j-1][3],train_m[j-1][4],train_m[j-1][5]))
							print("RNN_force\t\t:\t{:.5}\t\t{:.5}\t\t{:.5}".format(train_m[j-1][6],train_m[j-1][7],train_m[j-1][8]))
							print("\n")
							y1 = str(train_m[j-1][0])
							y2 = str(train_m[j-1][1])
							y3 = str(train_m[j-1][2])
							y4 = str(train_m[j-1][3])
							y5 = str(train_m[j-1][4])
							y6 = str(train_m[j-1][5])
							y7 = str(train_m[j-1][6])
							y8 = str(train_m[j-1][7])
							y9 = str(train_m[j-1][8])
							y10 = str(train_m[j-1][9])
							y11 = str(train_m[j-1][10])
							y12 = str(train_m[j-1][11])
							y13 = str(train_m[j-1][12])
							y14 = str(train_m[j-1][13])
							y15 = str(train_m[j-1][14])
							y16 = str(train_m[j-1][15])
							y17 = str(train_m[j-1][16])
							y18 = str(train_m[j-1][17])

							#y7 = str(train_m[j-1][6])
							#y8 = str(train_m[j-1][7])
							#y9 = str(train_m[j-1][8])
	
							msg = y1 + (",") + y2 + (",") + y3 + (",") + y4 + (",") + y5 + (",") + y6 + (",") + y7 + (",") + y8 + (",") + y9 + (",") + y10 + (",") + y11 + (",") + y12 + (",") + y13 + (",") + y14 + (",") + y15 + (",") + y16 + (",") + y17 + (",") + y18
							msg1 = bytes(msg,encoding='utf8')
							sock.send(msg1)


	finally:
		for sock in readfds:
			sock.close()
	#ShowTrajectory(predict,train_m)
	#np.savetxt("predict.csv",predict,delimiter=",")

	return

if __name__ == '__main__':
  main()
