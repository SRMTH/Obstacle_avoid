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

u_b,v_b = 177,138
u_g,v_g = 124,163
u_y,v_y = 60,110

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

	mask_blue = cv2.inRange(img_yuv, (np.array([0,u_b-45,v_b-45])), (np.array([255,u_b+45,v_b+45])))
	mask_green = cv2.inRange(img_yuv, (np.array([0,u_g-45,v_g-45])), (np.array([255,u_g+45,v_g+45])))
	mask_yellow = cv2.inRange(img_yuv, (np.array([0,u_y-45,v_y-45])), (np.array([255,u_y+45,v_y+45])))

	#FOR BLUE
	erode_b = cv2.erode(mask_blue,kernel,iterations = 1)
	dilate_b = cv2.dilate(erode_b,kernel,iterations = 1)
	image_b,contour_b,hierarchy_b = cv2.findContours(dilate_b,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)


	#FOR RED
	'''erode_r = cv2.erode(mask_green,kernel,iterations = 1)
	dilate_r = cv2.dilate(erode_r,kernel,iterations = 1)
	image_r,contour_g,hierarchy_r = cv2.findContours(dilate_r,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)'''

	#FOR YELLOW
	erode_y = cv2.erode(mask_yellow,kernel,iterations = 1)
	dilate_y = cv2.dilate(erode_y,kernel,iterations = 1)
	image_y,contour_y,hierarchy_y = cv2.findContours(dilate_y,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

	#FOR GREEN
	erode_g = cv2.erode(mask_green,kernel,iterations = 1)
	dilate_g = cv2.dilate(erode_g,kernel,iterations = 1)
	image_g,contour_g,hierarchy_g = cv2.findContours(dilate_g,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(frame, contour_b, -1, (0,255,0), 3)
	cv2.drawContours(frame, contour_y, -1, (0,255,0), 3)
	cv2.drawContours(frame, contour_g, -1, (0,255,0), 3)
		
	if cv2.waitKey(1) == 27:
		flag = 0
		
	if contour_b or contour_y or contour_g:
	 	cv2.imshow("sds",frame)
	
		if contour_b:
			cnt_b = max(contour_b, key = cv2.contourArea)
			x_b, y_b, w, h = cv2.boundingRect(cnt_b)
			cv2.rectangle(frame,(x_b,y_b),(x_b+w,y_b+h),(0,255,0),2)
			if x_b<x1+paramx and x_b>x1-paramx:
				return "blue detected"
		if contour_g:
			cnt_b = max(contour_g, key = cv2.contourArea)
			x_b, y_b, w, h = cv2.boundingRect(cnt_b)
			cv2.rectangle(frame,(x_b,y_b),(x_b+w,y_b+h),(0,255,0),2)
			if x_b<x1+paramx and x_b>x1-paramx:
				return "blue detected"


		if contour_y:
			cnt_y = max(contour_y, key = cv2.contourArea)
			x_y, y_y, w, h = cv2.boundingRect(cnt_y)
			cv2.rectangle(frame,(x_y,y_y),(x_y+w,y_y+h),(0,255,0),2)
			if x_y<x1+paramx and x_y>x1-paramx:
				if y_y>y1+paramy:
					return "tilt_d"
				if y_y<y1-paramy:
					return "tilt_u"
		
		return "safe"
	else:
			cv2.imshow("sds",frame)
			#not detected
			return "safe"



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







