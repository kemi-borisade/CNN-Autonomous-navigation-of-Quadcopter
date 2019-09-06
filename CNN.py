# -*- coding: utf-8 -*-
"""
Created on Sun Jul 05 15:01:58 2015

@author: ACSECKIN
"""


import vrep
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import imshow, show
import ctypes

commands = ["forward", "leftMove", "rightMove","rotateLeft","rotateRight","stop"] #declare label commands
model = models.load_model("/Users/model.h5") #load trained CNN model
vrep.simxFinish(-1) 

clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5) #set a connection ID with V-REP

	if clientID!=-1: #checks if there is a connection in place
    	print ('Connected to remote API server')
    	print ('Vision Sensor object handling')
    	res, v1 = vrep.simxGetObjectHandle(clientID, 'screenshotSensor', vrep.simx_opmode_oneshot_wait) 
    	print ('Getting first image')
    	err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v1, 0, vrep.simx_opmode_streaming)
    	while (vrep.simxGetConnectionId(clientID) != -1):
    		#gets image
        	err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v1, 0, vrep.simx_opmode_buffer)
  
        	if err == vrep.simx_return_ok: #if the image is gotten successfully convert the image to an array
            	img = np.array(image,dtype=np.uint8)
            	plt.imshow(img,origin='lower')
            	show() #display image
          		img = im.resize((224, 224), Image.ANTIALIAS)
            	img_array = np.expand_dims(im, axis=0)
           		
           		Data = img_array.astype('float32') #normalize data
				Data /=255

     			#predict image class and return class with highest probability
				prediction = model.predict(Data,steps=1,verbose=1) 
				predict_class = commands[np.argmax(prediction)] 
				print(predict_class )

				#send command classified to VREP

				if predict_class == forward:
					com = b'forward' #save the string in a variable
            		for_ubytes = (ctypes.c_ubyte * len(com)).from_buffer_copy(com) #convert to bytes
            		err = vrep.simxSetStringSignal(clientID,b"Comms",for_ubytes,vrep.simx_opmode_oneshot) #Send command

				elif predict_class == leftMove:
					comL = b'leftMove' #save the string in a variable
            		lefM_ubytes = (ctypes.c_ubyte * len(comL)).from_buffer_copy(comL) #convert to bytes
            		err = vrep.simxSetStringSignal(clientID,b"Comms",lefM_ubytes,vrep.simx_opmode_oneshot) #Send command


				elif predict_class == rightMove:
					comR = b'rightMove' #save the string in a variable
            		rightM_ubytes = (ctypes.c_ubyte * len(comR)).from_buffer_copy(comR) #convert to bytes
            		err = vrep.simxSetStringSignal(clientID,b"Comms",rightM_ubytes,vrep.simx_opmode_oneshot)#Send command


				elif predict_class == rotateRight:
					comrR = b'rotateRight' #save the string in a variable
            		rR_ubytes = (ctypes.c_ubyte * len(comrR)).from_buffer_copy(comrR) #convert to bytes
            		err = vrep.simxSetStringSignal(clientID,b"Comms",rR_ubytes,vrep.simx_opmode_oneshot)#Send command


				elif predict_class == rotateLeft:
					comrL = b'rotateLeft' #save the string in a variable
            		rL_ubytes = (ctypes.c_ubyte * len(comrL)).from_buffer_copy(comrL) #convert to bytes
            		err = vrep.simxSetStringSignal(clientID,b"Comms",rL_ubytes,vrep.simx_opmode_oneshot) #Send command


				else predict_class == stop
					comS = b'stop' #save the string in a variable
            		stop_ubytes = (ctypes.c_ubyte * len(comS)).from_buffer_copy(comS) #convert to bytes
            		err = vrep.simxSetStringSignal(clientID,b"Comms",stop_ubytes,vrep.simx_opmode_oneshot) #Send command

        elif err == vrep.simx_return_novalue_flag:
            print ("no image yet")
            pass
        else:
            print (err) #print the error
        
else:
  print ("Failed to connect to remote API Server")
  vrep.simxFinish(clientID)
       






