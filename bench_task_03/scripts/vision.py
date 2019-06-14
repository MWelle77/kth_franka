#!/usr/bin/env python
# license removed for brevity
#import rospy
import cv2
import ellipses as el
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.image_pub_e = rospy.Publisher("image_topic_e",Image)

    self.area_pub = rospy.Publisher('area_target', Int32, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    #ROI cordinates
    y=0
    x=275
    h=225
    w=225
    crop_frame = cv_image[y:y+h, x:x+w]
    yellow_pix=0
    #cv2.imshow('frame',crop_frame)
    #find the object (yellow) (not needed for pipline)
    # hsv_obj = cv2.cvtColor(crop_frame, cv2.COLOR_BGR2HSV) 
    # lower_yellow = np.array([20,100,100]) #cvScalar(20, 100, 100), cvScalar(30, 255, 255)
    # upper_yellow = np.array([30,255,255]) 
    # mask_obj = cv2.inRange(hsv_obj, lower_yellow, upper_yellow) 

    # res_obj = cv2.bitwise_and(crop_frame,crop_frame, mask= mask_obj) 

    #find the shirt (white)
    hsv_shirt = cv2.cvtColor(crop_frame, cv2.COLOR_BGR2HSV) 
    lower_white = np.array([0,0,160])
    upper_white = np.array([255,25,255])
    lower_yellow = np.array([20,100,100]) #cvScalar(20, 100, 100), cvScalar(30, 255, 255)
    upper_yellow = np.array([30,255,255]) 
    

    mask_shirt = cv2.inRange(hsv_shirt, lower_white, upper_white) 
    kernel = np.ones((3,3),np.uint8)
    opening = cv2.morphologyEx(mask_shirt, cv2.MORPH_OPEN, kernel)
    #morphological opening to get rid of outlyer pixels
    res_shirt = cv2.bitwise_and(crop_frame,crop_frame, mask= opening) 
    # fitt pixles in leastsquare ellipse format
    datax = []
    datay = []
    for x in range(np.array(opening).shape[0]):
        for y in range(np.array(opening).shape[0]):
            if(opening[x,y]==255):
                datax.append(x)
                datay.append(y)    
    data = []
    data.append(datax)
    data.append(datay)
    
    # debug
    # cv2.imshow('frame',crop_frame)
    # cv2.imshow('obj',res_obj)
    # cv2.imshow('shirt',opening)
    
    #do least square from: https://github.com/bdhammel/least-squares-ellipse-fitting
    #try:
    if(len(datax)>0):
        lsqe = el.LSqEllipse()
        lsqe.fit(data)
        center, width, height, phi = lsqe.parameters()

        #add the ellipse to the crop_frame
        cv2.ellipse(crop_frame,(int(center[1]),int(center[0])),(int(height),int(width)),np.rad2deg(-phi),0,360,255,5)
        #debug
        #cv2.ellipse(opening,(int(center[1]),int(center[0])),(int(height),int(width)),np.rad2deg(-phi),0,360,255,5)
        
        #get the insifde of ellipse mask
        e_mask=np.ones((w,h))*0
        e_mask=e_mask.astype('uint8')
        cv2.ellipse(e_mask,(int(center[1]),int(center[0])),(int(height),int(width)),np.rad2deg(-phi),0,360,255,-1)
        
        #filter
        res_elipse = cv2.bitwise_and(crop_frame,crop_frame, mask= e_mask) 
        res_elipse_hsv_obj = cv2.cvtColor(res_elipse, cv2.COLOR_BGR2HSV) 
        mask_obj_ellipse = cv2.inRange(res_elipse_hsv_obj, lower_yellow, upper_yellow) 
        #count yellow pixels inside ellipse
        yellow_pix=cv2.countNonZero(mask_obj_ellipse)

        #add information to vision screen
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(crop_frame,str(yellow_pix),(10,50), font, 1,(255,255,255),2,cv2.LINE_AA)
    else:
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(crop_frame,str(0),(10,50), font, 1,(255,255,255),2,cv2.LINE_AA)
        
        # Display the resulting frame (debug mode)
        #cv2.imshow('frame',crop_frame)
        #cv2.imshow('obj',res_obj)
        #cv2.imshow('shirt',opening)
        #cv2.imshow('emask',res_elipse)
        #cv2.imshow('mask_yellow_ellipse',mask_obj_ellipse)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
        #    break
    #except:
    #    print("ERROR")

    # (rows,cols,channels) = cv_image.shape
    # if cols > 60 and rows > 60 :
    #   cv2.circle(cv_image, (50,50), 10, 255)
    # cv2.imshow("Image window", cv_image)
    # cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(crop_frame, "bgr8"))
      self.area_pub.publish(yellow_pix)
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)



# while(True):

#     cap = cv2.VideoCapture(0)
#     ret, frame = cap.read()
#     cv2.imshow('frame',frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break

# if __name__ == '__main__': 

    
#     print("starting")
#     # publisher 
#     #pub_area = rospy.Publisher('target_area', String, queue_size=10)
#     pub_image = rospy.Publisher("vision_view",Image,queue_size=1)
#     rospy.init_node('vision', anonymous=True)
#     rate = rospy.Rate(10) # 10hz



#     while not rospy.is_shutdown():
#         print("starting")

#         ret, frame = cap.read()

#         #ROI cordinates
#         y=0
#         x=225
#         h=225
#         w=225
#         crop_frame = frame[y:y+h, x:x+w]
#         cv2.imshow('frame',crop_frame)
#         #find the object (yellow) (not needed for pipline)
#         # hsv_obj = cv2.cvtColor(crop_frame, cv2.COLOR_BGR2HSV) 
#         # lower_yellow = np.array([20,100,100]) #cvScalar(20, 100, 100), cvScalar(30, 255, 255)
#         # upper_yellow = np.array([30,255,255]) 
#         # mask_obj = cv2.inRange(hsv_obj, lower_yellow, upper_yellow) 

#         # res_obj = cv2.bitwise_and(crop_frame,crop_frame, mask= mask_obj) 

#         #find the shirt (white)
#         hsv_shirt = cv2.cvtColor(crop_frame, cv2.COLOR_BGR2HSV) 
#         lower_white = np.array([0,0,160])
#         upper_white = np.array([255,25,255])

#         mask_shirt = cv2.inRange(hsv_shirt, lower_white, upper_white) 
#         kernel = np.ones((3,3),np.uint8)
#         opening = cv2.morphologyEx(mask_shirt, cv2.MORPH_OPEN, kernel)
#         #morphological opening to get rid of outlyer pixels
#         res_shirt = cv2.bitwise_and(crop_frame,crop_frame, mask= opening) 
#         # fitt pixles in leastsquare ellipse format
#         datax = []
#         datay = []
#         for x in range(np.array(opening).shape[0]):
#             for y in range(np.array(opening).shape[0]):
#                 if(opening[x,y]==255):
#                     datax.append(x)
#                     datay.append(y)    
#         data = []
#         data.append(datax)
#         data.append(datay)
        
#         # debug
#         # cv2.imshow('frame',crop_frame)
#         # cv2.imshow('obj',res_obj)
#         # cv2.imshow('shirt',opening)
        
#         #do least square from: https://github.com/bdhammel/least-squares-ellipse-fitting
#         try:
#             lsqe = el.LSqEllipse()
#             lsqe.fit(data)
#             center, width, height, phi = lsqe.parameters()

#             #add the ellipse to the crop_frame
#             cv2.ellipse(crop_frame,(int(center[1]),int(center[0])),(int(height),int(width)),np.rad2deg(-phi),0,360,255,5)
#             #debug
#             #cv2.ellipse(opening,(int(center[1]),int(center[0])),(int(height),int(width)),np.rad2deg(-phi),0,360,255,5)
            
#             #get the insifde of ellipse mask
#             e_mask=np.ones((w,h))*0
#             e_mask=e_mask.astype('uint8')
#             cv2.ellipse(e_mask,(int(center[1]),int(center[0])),(int(height),int(width)),np.rad2deg(-phi),0,360,255,-1)
            
#             #filter
#             res_elipse = cv2.bitwise_and(crop_frame,crop_frame, mask= e_mask) 
#             res_elipse_hsv_obj = cv2.cvtColor(crop_frame, cv2.COLOR_BGR2HSV) 
#             mask_obj_ellipse = cv2.inRange(res_elipse_hsv_obj, lower_yellow, upper_yellow) 
#             #count yellow pixels inside ellipse
#             yellow_pix=cv2.countNonZero(mask_obj_ellipse)

#             #add information to vision screen
#             font = cv2.FONT_HERSHEY_SIMPLEX
#             cv2.putText(crop_frame,str(yellow_pix),(10,50), font, 1,(255,255,255),2,cv2.LINE_AA)
            
#             # publish ros image
#             try:
#                 pub_image.publish(self.bridge.cv2_to_imgmsg(crop_frame, "bgr8"))
#             except CvBridgeError as e:
#               print(e)
#             # Display the resulting frame (debug mode)
#             cv2.imshow('frame',crop_frame)
#             #cv2.imshow('obj',res_obj)
#             #cv2.imshow('shirt',opening)
#             #cv2.imshow('emask',res_elipse)
#             #cv2.imshow('mask_yellow_ellipse',mask_obj_ellipse)
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break
#         except:
#             print("ERROR")

# # When everything done, release the capture
# cap.release()
# cv2.destroyAllWindows()
