#!/usr/bin/env python3

def correct_points(low_plane=.0,high_plane=0.2):

    #Corrects point clouds "perspective" i.e. Reference frame head is changed to reference frame map
    data = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)
    np_data=ros_numpy.numpify(data)
    trans,rot=listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0)) 
    
    eu=np.asarray(tf.transformations.euler_from_quaternion(rot))
    t=TransformStamped()
    rot=tf.transformations.quaternion_from_euler(-eu[1],0,0)
    t.header.stamp = data.header.stamp
    
    t.transform.rotation.x = rot[0]
    t.transform.rotation.y = rot[1]
    t.transform.rotation.z = rot[2]
    t.transform.rotation.w = rot[3]

    cloud_out = do_transform_cloud(data, t)
    np_corrected=ros_numpy.numpify(cloud_out)
    corrected=np_corrected.reshape(np_data.shape)

    img= np.copy(corrected['y'])

    img[np.isnan(img)]=2
    #img3 = np.where((img>low)&(img< 0.99*(trans[2])),img,255)
    img3 = np.where((img>0.99*(trans[2])-high_plane)&(img< 0.99*(trans[2])-low_plane),img,255)
    return img3

def cloud_cb(msg):    
    global points_data
    global image_data
    points_data = ros_numpy.numpify(msg)
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
      

def gaze_point(x,y,z):
    
    
    
    head_pose = head.get_current_joint_values()
    head_pose[0]=0.0
    head_pose[1]=0.0
    head.set_joint_value_target(head_pose)
    head.go()
    
    trans , rot = listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0)) #
    
 
    
    e =tf.transformations.euler_from_quaternion(rot)
    

    x_rob,y_rob,z_rob,th_rob= trans[0], trans[1] ,trans[2] ,  e[2]


    D_x=x_rob-x
    D_y=y_rob-y
    D_z=z_rob-z

    D_th= np.arctan2(D_y,D_x)
    print('relative to robot',(D_x,D_y,np.rad2deg(D_th)))

    pan_correct= (- th_rob + D_th + np.pi) % (2*np.pi)

    if(pan_correct > np.pi):
        pan_correct=-2*np.pi+pan_correct
    if(pan_correct < -np.pi):
        pan_correct=2*np.pi+pan_correct

    if ((pan_correct) > .5 * np.pi):
        print ('Exorcist alert')
        pan_correct=.5*np.pi
    head_pose[0]=pan_correct
    tilt_correct=np.arctan2(D_z,np.linalg.norm((D_x,D_y)))

    head_pose [1]=-tilt_correct
    
    
    
    head.set_joint_value_target(head_pose)
    succ=head.go()
    return succ


########################### Setup ######################################33
#get_ipython().run_line_magic('matplotlib', 'inline')
from matplotlib import pyplot as plt
import numpy as np
import ros_numpy
import rospy
import tf
from gazebo_ros import gazebo_interface
from sensor_msgs.msg import LaserScan, PointCloud2, Image
from geometry_msgs.msg import Pose, Quaternion ,TransformStamped
from cv_bridge import CvBridge
import moveit_commander
import moveit_msgs.msg
import sys
import cv2
import os
import math
import time
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
#from class_rgbd import RGBD
########################################################################################

rospy.init_node("recognition")
rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2, cloud_cb)


#rgbd = RGBD()
###clase rgbd para obtener mensajes de nube de puntos y de imagen


listener = tf.TransformListener()
broadcaster= tf.TransformBroadcaster()


head = moveit_commander.MoveGroupCommander('head')
#whole_body=moveit_commander.MoveGroupCommander('whole_body_weighted')
arm =  moveit_commander.MoveGroupCommander('arm')


arm.set_named_target('go')
arm.go()
inicia=rospy.Time.now().to_sec()

#gaze_point(0.0,0.0,0.0) #definimos la direccion de la cabeza
#gaze_point(0.539459,1.12990,0.787205) #definimos la direccion de la cabeza


#head.go(np.array((0,-.15*np.pi)))

########################    Obtencion de nubes de puntos ##############################################

rospy.sleep(5)
points= points_data #Obtenemos una nube de puntos, similar a la nube de puntos corregida
image= image_data  #Obtenemos la imagen

#print (points.shape)
#print (image.shape)
#print(image.dtype)


# una matriz (arreglo tipo numpy) 480px por 640 px 3 canales
#plt.imshow(image_data)
#plt.show()

#################################### Conversion de RGB a BGR ##########################################

im_bgr=cv2.cvtColor(image, cv2.COLOR_RGB2BGR) #cambiamos la imagen a BGR
#plt.imshow(im_bgr)
#plt.show()

im_hsv= cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)
plt.imshow(im_hsv)
plt.show()

#plt.imshow(im_hsv[:,:,0]) #codificacion de color en el canal 0 Hue
#plt.show()

#plt.imshow(points['z'])
#plt.show()

#plt.imshow(points['z'])
#plt.show()

#plt.imshow(points['z'],cmap='prism')
#plt.show()
######################################## Valores de Hue Value Saturation #####################################3
h_min=200
h_max=255
#h_min=220
#h_max=255
region = (im_hsv > h_min) & (im_hsv < h_max)
idx,idy=np.where(region[:,:,1] )
mask= np.zeros((480,640))
mask[idx,idy]=255
plt.imshow(mask ,cmap='gray')
plt.show()

kernel = np.ones((5, 5), np.uint8)
eroded_mask=cv2.erode(mask,kernel)
dilated_mask=cv2.dilate(eroded_mask,kernel)

plt.imshow(dilated_mask,cmap='gray')
plt.show()

####################################### Contornos ##############################################
contours, hierarchy = cv2.findContours(dilated_mask.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
print(len(contours))

for contour in contours:
    M = cv2.moments(contour) #MOMENTOS ESADISTICOS DE LA IMAGEN
    
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    boundRect = cv2.boundingRect(contour)
    image2=cv2.rectangle(im_hsv,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (255,255,255), 2)
    cv2.circle(image2, (cX, cY), 5, (255, 255, 255), -1)
    cv2.putText(image2, "centroid_"+str(cX)+','+str(cY)    ,    (cX - 50, cY - 25)   ,cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

plt.imshow(cv2.cvtColor( image2, cv2.COLOR_HSV2RGB))
plt.show()

coordenadas=[]

for contour in contours:
    xyz=[]
    M = cv2.moments(contour) #MOMENTOS ESADISTICOS DE LA IMAGEN
    
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    boundRect = cv2.boundingRect(contour)

    for jy in range (boundRect[0], boundRect[0]+boundRect[2]):
        for ix in range(boundRect[1], boundRect[1]+boundRect[3]):
            aux=(np.asarray((points['x'][ix,jy],points['y'][ix,jy],points['z'][ix,jy])))
            if np.isnan(aux[0]) or np.isnan(aux[1]) or np.isnan(aux[2]):
                'reject point'
            else:
                xyz.append(aux)

    xyz=np.asarray(xyz)
    cent=xyz.mean(axis=0)
    coordenadas.append(cent)

    

######################################### Publicacion de Tf respecto a la cabeza ###########################################

coordenadas=np.array(coordenadas)
print(coordenadas)

x,y,z= coordenadas[0]
#o,p,q= coordenadas[1]

if np.isnan(x) or np.isnan(y) or np.isnan(z):
    print('nan')
else:
    broadcaster.sendTransform((x,y,z),(0,0,0,1), rospy.Time.now(), 'Object',"head_rgbd_sensor_link")

'''
if np.isnan(x) or np.isnan(y) or np.isnan(z):
    print('nan')
else:
    broadcaster.sendTransform((o,p,q),(0,0,0,1), rospy.Time.now(), 'Object2',"head_rgbd_sensor_link")
'''
#x,y,z=cent
#if np.isnan(x) or np.isnan(y) or np.isnan(z):
#    print('nan')
#else:
#    broadcaster.sendTransform((x,y,z),(0,0,0,1), rospy.Time.now(), 'Object',"head_rgbd_sensor_link")

rospy.sleep(2)
####################################### Escuchar a las transformadas #########################################################

k=listener.lookupTransform('map','Object',rospy.Time(0))
'''
z=listener.lookupTransform('map','Object2',rospy.Time(0))
'''

################################# Publicar transformada respecto al mapa #####################################################

broadcaster.sendTransform(k[0],(0,0,0,1), rospy.Time.now(), 'Object_fix','map')
'''
broadcaster.sendTransform(z[0],(0,0,0,1), rospy.Time.now(), 'Object_fix2','map')
'''

##################################### Imprime los valores de las transformadas ################################################
rospy.sleep(2)
obj2=listener.lookupTransform('map','Object_fix',rospy.Time(0))
'''
obj3=listener.lookupTransform('map','Object_fix2',rospy.Time(0))
'''
print('Coordenadas respecto al mapa del primero objeto ',obj2[0])

'''
print('Coordenadas respecto al mapa del segundo objeto ',obj3[0])
'''
######################################## Guarda en archivo txt #################################################33
f=open('/home/cire2022/coordenadas.txt','w')
f.write("Coordenadas respecto al mapa")
f.write('\n')
f.write("Las coordenadas del primero objeto object_fix son")
f.write('\n')
f.write(', '.join(map(str,obj2[0])))
''''
f.write('\n')
f.write("Las coordenadas del segundo objeto object_fix2 son")
f.write('\n')
f.write(', '.join(map(str,obj3[0])))
'''
f.close()

duracion=rospy.Time.now().to_sec()-inicia
print('La simulacion tuvo una duracion de: ',duracion,' sec.')
######################### Trash ########################################









#corrected= correct_points(0.60, 0.7)
#corrected= correct_points(0.1,0.2)


#plt.imshow(corrected)
#plt.show()

#contours, hierarchy = cv2.findContours(corrected.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
#print(len(contours))



#####################################################
#kernel=np.ones((5,5),np.uint8)
#eroded_mask= cv2.erode(mask, kernel)
#dilated_mask=cv2.dilate(eroded_mask)

