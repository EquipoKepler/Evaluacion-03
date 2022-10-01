#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import tf2_ros
import time
from sensor_msgs.msg import LaserScan

print("Se cargaron las librerias")

def coord_pro(msg2):
    global objetivox
    global objetivoy
    objetivox=msg2.pose.position.x
    objetivoy=msg2.pose.position.y

#############################################################
def callback_scan(msg):

    global obstacle_detected1
    global obstacle_detected2
    global rango1#derecha del robot
    global rango2#izquierda del robot
    rango1=msg.ranges[90]
    rango2=msg.ranges[629]
    obstacle_detected1 = rango1<0.3
    obstacle_detected2 = rango2<0.3
    return

#####################################################################

def get_coords():
    got_transform= False
    
    while not got_transform:
        try:
            trans=tfBuffer.lookup_transform('map','base_link',rospy.Time())
            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
     
####################################################################

rospy.init_node('obtener_coordenadas')
rospy.Subscriber("/hsrb/base_scan", LaserScan, callback_scan)
rospy.Subscriber("/meta_competencia", PoseStamped,coord_pro)
pub_cmd_vel = rospy.Publisher("/hsrb/command_velocity", Twist, queue_size=10)
loop = rospy.Rate(10)

#######################################################################
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
coords_act=get_coords()
print("Las coordenadas actuales en x son: ", coords_act.transform.translation.x)
print("Las coordenadas actuales en y son: ", coords_act.transform.translation.y)
#################################################################################

start_ejecution=rospy.Time.now().to_sec() #Tiempo en el que inicia el programa

#############################################################################
obstacle_detected1=0
obstacle_detected2=0
#####################################################################################
msg_cmd_vel=Twist()

while rospy.Time.now().to_sec()-start_ejecution<2 and not rospy.is_shutdown():

    if obstacle_detected1 and obstacle_detected2:
        #print("un objeto se detecto, para")
        msg_cmd_vel.linear.y=0
        msg_cmd_vel.linear.x=0
        pub_cmd_vel.publish(msg_cmd_vel)
    else:
        #print("no hay objetos cercanos, sigue")
        msg_cmd_vel.linear.y=0.1
        msg_cmd_vel.linear.x=0
        pub_cmd_vel.publish(msg_cmd_vel)

start_ejecution2=rospy.Time.now().to_sec()

while rospy.Time.now().to_sec()-start_ejecution2<20 and not rospy.is_shutdown():

    if obstacle_detected1 and obstacle_detected2:
        #print("un objeto se detecto, para")
        msg_cmd_vel.linear.x=0
        msg_cmd_vel.linear.y=0
        pub_cmd_vel.publish(msg_cmd_vel)
    
    else:
        #print("no hay objetos cercanos, sigue")
        msg_cmd_vel.linear.x=-0.1
        msg_cmd_vel.linear.y=0
        pub_cmd_vel.publish(msg_cmd_vel)   

start_ejecution4=rospy.Time.now().to_sec()

while rospy.Time.now().to_sec()-start_ejecution4<6 and not rospy.is_shutdown():

    if obstacle_detected1 and obstacle_detected2:
        #print("un objeto se detecto, para")
        msg_cmd_vel.linear.x=0
        msg_cmd_vel.linear.y=0
        pub_cmd_vel.publish(msg_cmd_vel)
    
    else:
        #print("no hay objetos cercanos, sigue")
        msg_cmd_vel.linear.x=0
        msg_cmd_vel.linear.y=0
        msg_cmd_vel.angular.z=0.1
        pub_cmd_vel.publish(msg_cmd_vel)   
     
start_ejecution3=rospy.Time.now().to_sec()

while rospy.Time.now().to_sec()-start_ejecution3<10 and not rospy.is_shutdown():

    if obstacle_detected1 and obstacle_detected2:
        #print("un objeto se detecto, para")
        msg_cmd_vel.linear.x=0
        msg_cmd_vel.linear.y=0
        pub_cmd_vel.publish(msg_cmd_vel)
    
    else:
        #print("no hay objetos cercanos, sigue")
        msg_cmd_vel.linear.x=0.1
        msg_cmd_vel.linear.y=0
        pub_cmd_vel.publish(msg_cmd_vel)    
start_ejecution5=rospy.Time.now().to_sec()
rospy.sleep(1)
while rospy.Time.now().to_sec()-start_ejecution5<10 and not rospy.is_shutdown():

    if obstacle_detected1 and obstacle_detected2:
        #print("un objeto se detecto, para")
        msg_cmd_vel.linear.x=0
        msg_cmd_vel.linear.y=0
        pub_cmd_vel.publish(msg_cmd_vel)
    
    else:
        #print("no hay objetos cercanos, sigue")
        msg_cmd_vel.linear.x=0.1
        msg_cmd_vel.linear.y=0
        pub_cmd_vel.publish(msg_cmd_vel)   
     
##start_ejecution6=rospy.Time.now().to_sec()
##rospy.sleep(1)
##while rospy.Time.now().to_sec()-start_ejecution6<4 and not rospy.is_shutdown():

  ##  if obstacle_detected1 and obstacle_detected2:
    ##  #print("un objeto se detecto, para")
    ##  msg_cmd_vel.linear.x=0
        ##msg_cmd_vel.linear.y=0
        ##pub_cmd_vel.publish(msg_cmd_vel)
    
   ## else:
    ##   #print("no hay objetos cercanos, sigue")
    ## msg_cmd_vel.linear.x=0
    ##  msg_cmd_vel.linear.y=0
    ##   msg_cmd_vel.angular.z=0.1
    ## pub_cmd_vel.publish(msg_cmd_vel)   
     

################################################################################################
print("El tiempo de ejecucion del programa es: ",rospy.Time.now().to_sec()-start_ejecution)
print("La posicion objetivo en x es: ",objetivox)
print("La posicion objetivo en y es: ",objetivoy)
print("Las coordenadas del robot en x son: ", coords_act.transform.translation.x)
print("Las coordenadas del robot en y son: ", coords_act.transform.translation.y)

   


#if name == "main":
#   try:
#       main()
#   except rospy.ROSInterruptException:
#       pass
