#!/usr/bin/env python
import cv2
import numpy as np
import time
import rospy
from std_msgs.msg import String


try:
	index = sys.argv[1]
except:
	index = 1 



cap = cv2.VideoCapture(index)

kernel = np.ones((5,5),np.uint8)

width = cap.get(3)
height= cap.get(4)

x,y = 0,0
flag = 1
cv2.namedWindow("YUV")
paramx,paramy = 150,150


x1,y1 = int(width/2),int(height/2)


def detect():

	global cap,y,u,v,kernel,width,height,flag,area,state,paramx,paramy,x1,y1,stop
	

		
	ret,frame = cap.read()
	img_yuv = cv2.cvtColor(frame,cv2.COLOR_BGR2YUV)

	mask_blue = cv2.inRange(img_yuv, (np.array([0,u_blue-30,v_blue-30])), (np.array([255,u_blue+30,v_blue+30])))
	mask_red = cv2.inRange(img_yuv, (np.array([0,u_red-30,v_red-30])), (np.array([255,u_red+30,v_red+30])))
	mask_yellow = cv2.inRange(img_yuv, (np.array([0,u_yellow-30,v_yellow-30])), (np.array([255,u_yellow+30,v_yellow+30])))

	#FOR BLUE
	erode_b = cv2.erode(mask_blue,kernel,iterations = 1)
	dilate_b = cv2.dilate(erode_b,kernel,iterations = 1)
	image_b,contour_b,hierarchy_b = cv2.findContours(dilate_b,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)


	#FOR RED
	erode_r = cv2.erode(mask_red,kernel,iterations = 1)
	dilate_r = cv2.dilate(erode_r,kernel,iterations = 1)
	image_r,contour_r,hierarchy_r = cv2.findContours(dilate_r,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

	#FOR YELLOW
	erode_y = cv2.erode(mask_yellow,kernel,iterations = 1)
	dilate_y = cv2.dilate(erode_y,kernel,iterations = 1)
	image_y,contour_y,hierarchy_y = cv2.findContours(dilate_y,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

	#FOR GREEN
	erode_g = cv2.erode(mask_green,kernel,iterations = 1)
	dilate_g = cv2.dilate(erode_g,kernel,iterations = 1)
	image_g,contour_g,hierarchy_g = cv2.findContours(dilate_g,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

		
	if cv2.waitKey(1) == 27:
		flag = 0
		
	if contour_b or contour_y or contour_r or contour_g:
	 
	
		if contour_b:
			cnt_b = max(contour_b, key = cv2.contourArea)
			x_b, y_b, w, h = cv2.boundingRect(cnt_b)
			msg = ["blue", str(x_b), str(y_b)]

		if contour_g:
			cnt_b = max(contour_b, key = cv2.contourArea)
			x_b, y_b, w, h = cv2.boundingRect(cnt_b)
			msg = ["blue", str(x_b), str(y_b)]

		if contour_r:
			cnt_r = max(contour_r, key = cv2.contourArea)
			x_r, y_r, w, h = cv2.boundingRect(cnt_r)
			msg = ["red", str(x_r), str(y_r)]

		if contour_y:
			cnt_y = max(contour_y, key = cv2.contourArea)
			x_y, y_y, w, h = cv2.boundingRect(cnt_y)
			msg = ["yellow", str(x_y), str(y_y)]

	else:
			#not detected
			msg =  ["not detected",'0','0']

	return " ".join(msg)


def talker() :
	global stop
	pub=rospy.Publisher('detect',String,queue_size=1)
	rospy.init_node('talker',anonymous=True) 
	rate=rospy.Rate(10)

	while not rospy.is_shutdown() or flag == 0 :
		
		msg = detect()
		pub.publish(msg)


if __name__=="__main__" :
	try :
		talker()
	except rospy.ROSInterruptException :
		cap.release()
		cv2.destroyAllWindows()






