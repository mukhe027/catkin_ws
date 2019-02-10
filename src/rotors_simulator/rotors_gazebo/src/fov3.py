#!/usr/bin/env python
"""
FOV detection for each drone within which another drone must fall
"""

import roslib
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Float64
from std_msgs.msg import Float32
import random
#no need of declaration of variables
#from pozyx_ros_examples.msg import DeviceRange
from time import sleep, time
from geometry_msgs.msg import Point, Pose, Quaternion
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import pypozyx
import numpy as np
import math

agents = 4
loc_agnt = 1
#pyramid 1 coordinates 
po_1 = [0 ,0 ,0 ,1]
pa_1 = [-3 ,-3 ,3 ,1]
pb_1 = [-3 , 3 ,3 ,1]
pc_1 = [-3 ,3 ,-3 ,1]
pd_1 = [-3 ,-3 ,-3 ,1]
ps_1 = [-2 ,0 ,0 ,1]
#pyramid 2 coordinates 
po_2 = [0 ,0 ,0 ,1]
pa_2 = [3 ,-3 ,3 ,1]
pb_2 = [3 , 3 ,3 ,1]
pc_2 = [3 ,3 ,-3 ,1]
pd_2 = [3 ,-3 ,-3 ,1]
ps_2 = [2 ,0 ,0 ,1]
#pyramid 3 coordinates 
po_3 = [0 ,0 ,0 ,1]
pa_3 = [3 ,-3 ,3 ,1]
pb_3 = [-3 ,-3 ,3 ,1]
pc_3 = [-3 ,-3 ,-3 ,1]
pd_3 = [3 ,-3 ,-3 ,1]
ps_3 = [0 ,-2 ,0 ,1]
#pyramid 4 coordinates 
po_4 = [0 ,0 ,0 ,1]
pa_4 = [-3 ,3 ,3 ,1]
pb_4 = [3 , 3 ,3 ,1]
pc_4 = [3 ,3 ,-3 ,1]
pd_4 = [-3 ,3 ,-3 ,1]
ps_4 = [0 ,2 ,0 ,1]
pos_list = np.zeros((3, 3))
D_1_2_s = np.zeros(5) 
D_2_2_s = np.zeros(5) 
D_3_2_s = np.zeros(5) 
D_4_2_s = np.zeros(5) 
D_1_2 = np.zeros(5) 
D_2_2 = np.zeros(5) 
D_3_2 = np.zeros(5) 
D_4_2 = np.zeros(5) 
bool_data = np.zeros(5)
dist_p1_d2 = np.zeros(4) 
dist_p2_d2 = np.zeros(4) 
dist_p3_d2 = np.zeros(4) 
dist_p4_d2 = np.zeros(4) 
dist_comp = np.zeros(4) 
z_G = 1.2 ##in mm
global adj 
adj = np.array([0.0,0.0,0.0,0.0],dtype=np.float32)
#euc_dist = 0.0 #maybe should not define it as global variable
def pos_2(pos2_data): 
   
    pos2_x = pos2_data.position.x
    pos2_y = pos2_data.position.y
    ornt2_x = pos2_data.orientation.x
    ornt2_y = pos2_data.orientation.y
    ornt2_z = pos2_data.orientation.z
    ornt2_w = pos2_data.orientation.w
    pos_list[0,0] = pos2_x
    pos_list[0,1] = pos2_y
    pos_list[0,2] = pos2_data.position.z
    return pos_list
def pos_3(pos3_data):    
    pos3_x = pos3_data.position.x
    pos3_y = pos3_data.position.y
    ornt3_x = pos3_data.orientation.x
    ornt3_y = pos3_data.orientation.y
    ornt3_z = pos3_data.orientation.z
    ornt3_w = pos3_data.orientation.w
    pos_list[1,0] = pos3_x
    pos_list[1,1] = pos3_y
    pos_list[1,2] = pos3_data.position.z
    return pos_list
def pos_4(pos4_data):
    pos4_x = pos4_data.position.x
    pos4_y = pos4_data.position.y 
    ornt4_x = pos4_data.orientation.x
    ornt4_y = pos4_data.orientation.y
    ornt4_z = pos4_data.orientation.z
    ornt4_w = pos4_data.orientation.w
    pos_list[2,0] = pos4_x
    pos_list[2,1] = pos4_y
    pos_list[2,2] = pos4_data.position.z
    return pos_list
def fov(data):
    """
    high level support for doing this and that.
    """
    
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    pos_x = data.position.x
    pos_y = data.position.y
    pos_z = data.position.z
    q_0 = data.orientation.x
    q_1 = data.orientation.y
    q_2 = data.orientation.z
    q_3 = data.orientation.w
    #Currently considering orientation with a 3D axis but available data is for only 2D
    '''T_AL = [[q_0**2+q_1**2-q_2**2-q_3**2 ,2*((q_1*q_2)- (q_0*q_3)) ,2*((q_0*q_2)+ (q_1*q_3)) ,pos_x],
     [2*((q_0*q_3)+ (q_1*q_2)) ,q_0**2-q_1**2+q_2**2-q_3**2 ,2*((q_2*q_3)- (q_0*q_1)) ,pos_y ],
     [2*((q_1*q_3)- (q_0*q_2)) ,2*((q_0*q_1)+ (q_2*q_3)) , q_0**2-q_1**2-q_2**2+q_3**2  ,pos_z],
     [0 ,0 ,0 ,1]]'''
    #Transpose of the above matrix, only the rotation matrix part of the Translation matrix
    T_AL = [[q_0**2+q_1**2-q_2**2-q_3**2 , 2*((q_0*q_3)+ (q_1*q_2)),2*((q_1*q_3)- (q_0*q_2)) ,pos_x],
     [2*((q_1*q_2)- (q_0*q_3)) ,q_0**2-q_1**2+q_2**2-q_3**2 ,2*((q_0*q_1)+ (q_2*q_3)) ,pos_y ],
     [2*((q_0*q_2)+ (q_1*q_3)) ,2*((q_2*q_3)- (q_0*q_1)) , q_0**2-q_1**2-q_2**2+q_3**2  ,pos_z],
     [0 ,0 ,0 ,1]]
    #Now will need to multiply Transform Matrix with 5 points of 4 Pyramids of FOV
    #Matrix Multiplication of first pyramid for global coordinates
    pog_1 = np.matmul(T_AL ,po_1)
    pag_1 = np.matmul(T_AL ,pa_1)
    pbg_1 = np.matmul(T_AL ,pb_1)
    pcg_1 = np.matmul(T_AL ,pc_1)
    pdg_1 = np.matmul(T_AL ,pd_1)
    psg_1 = np.matmul(T_AL ,ps_1)
   
    #Matrix Multiplication of first pyramid's top plane normal and d value
    pvtop_1 = np.subtract(pog_1,pag_1)
    pvtop_2 = np.subtract(pog_1,pbg_1) 
    norm_1 = np.cross(pvtop_1[0:3], pvtop_2[0:3]) 
    d_1 = np.matmul(norm_1.T ,pog_1[0:3])#check if this method gives a scalar value
    pvside_1 = np.subtract(pog_1,pcg_1)
    pvside_2 = np.subtract(pog_1,pbg_1)
    norm_2 = np.cross(pvside_1[0:3], pvside_2[0:3]) 
    d_2 = np.matmul(norm_2.T ,pog_1[0:3])

    pvbot_1 = np.subtract(pog_1,pcg_1)
    pvbot_2 = np.subtract(pog_1,pdg_1)
    norm_3 = np.cross(pvbot_1[0:3], pvbot_2[0:3]) 
    d_3 = np.matmul(norm_3.T ,pog_1[0:3])

    pvside_11 = np.subtract(pog_1,pag_1)
    pvside_22 = np.subtract(pog_1,pdg_1)
    norm_4 = np.cross(pvside_11[0:3], pvside_22[0:3]) 
    d_4 = np.matmul(norm_4.T ,pog_1[0:3])

    pvfront_1 = np.subtract(pbg_1,pag_1)
    pvfront_2 = np.subtract(pbg_1,pcg_1)
    norm_5 = np.cross(pvfront_1[0:3], pvfront_2[0:3]) 
    d_5 = np.matmul(norm_5.T ,pag_1[0:3])
   

   
    ##################################################################
    ##################################################################
    #Matrix Multiplication of 2nd pyramid
    pog_2 = np.matmul(T_AL ,po_2)
    pag_2 = np.matmul(T_AL ,pa_2)
    pbg_2 = np.matmul(T_AL ,pb_2)
    pcg_2 = np.matmul(T_AL ,pc_2)
    pdg_2 = np.matmul(T_AL ,pd_2)
    psg_2 = np.matmul(T_AL ,ps_2)
    #Matrix Multiplication of second pyramid's top plane normal and d value
    pvtop_2_1 = np.subtract(pog_2,pag_2)
    pvtop_2_2 = np.subtract(pog_2,pbg_2) 
    norm_2_1 = np.cross(pvtop_2_1[0:3], pvtop_2_2[0:3]) 
    d_2_1 = np.matmul(norm_2_1.T ,pog_2[0:3])
    
    pvside_2_1 = np.subtract(pog_2,pcg_2)
    pvside_2_2 = np.subtract(pog_2,pbg_2)
    norm_2_2 = np.cross(pvside_2_1[0:3], pvside_2_2[0:3]) 
    d_2_2 = np.matmul(norm_2_2.T ,pog_2[0:3])

    pvbot_2_1 = np.subtract(pog_2,pcg_2) 
    pvbot_2_2 = np.subtract(pog_2,pdg_2) 
    norm_2_3 = np.cross(pvbot_2_1[0:3], pvbot_2_2[0:3]) 
    d_2_3 = np.matmul(norm_2_3.T ,pog_2[0:3])

    pvside_2_11 = np.subtract(pog_2,pag_2) 
    pvside_2_22 = np.subtract(pog_2,pdg_2) 
    norm_2_4 = np.cross(pvside_2_11[0:3], pvside_2_22[0:3]) 
    d_2_4 = np.matmul(norm_2_4.T ,pog_2[0:3])

    pvfront_2_1 = np.subtract(pbg_2,pag_2)
    pvfront_2_2 = np.subtract(pbg_2,pcg_2)
    norm_2_5 = np.cross(pvfront_2_1[0:3], pvfront_2_2[0:3]) 
    d_2_5 = np.matmul(norm_2_5.T ,pag_2[0:3])
    ##################################################################
    ##################################################################
    #Matrix Multiplication of 3rd pyramid
    pog_3 = np.matmul(T_AL ,po_3)
    pag_3 = np.matmul(T_AL ,pa_3)
    pbg_3 = np.matmul(T_AL ,pb_3)
    pcg_3 = np.matmul(T_AL ,pc_3)
    pdg_3 = np.matmul(T_AL ,pd_3)
    psg_3 = np.matmul(T_AL ,ps_3)
    #Matrix Multiplication of third pyramid's top plane normal and d value
    pvtop_3_1 = np.subtract(pog_3,pag_3) 
    pvtop_3_2 = np.subtract(pog_3,pbg_3)
    norm_3_1 = np.cross(pvtop_3_1[0:3], pvtop_3_2[0:3]) 
    d_3_1 = np.matmul(norm_3_1.T,pog_3[0:3])
    
    pvside_3_1 = np.subtract(pog_3,pcg_3) 
    pvside_3_2 = np.subtract(pog_3,pbg_3)
    norm_3_2 = np.cross(pvside_3_1[0:3], pvside_3_2[0:3]) 
    d_3_2 = np.matmul(norm_3_2.T ,pog_3[0:3])

    pvbot_3_1 = np.subtract(pog_3,pcg_3)
    pvbot_3_2 = np.subtract(pog_3,pdg_3)
    norm_3_3 = np.cross(pvbot_3_1[0:3], pvbot_3_2[0:3]) 
    d_3_3 = np.matmul(norm_3_3.T ,pog_3[0:3])

    pvside_3_11 = np.subtract(pog_3,pag_3) 
    pvside_3_22 = np.subtract(pog_3,pdg_3)
    norm_3_4 = np.cross(pvside_3_11[0:3], pvside_3_22[0:3]) 
    d_3_4 = np.matmul(norm_3_4.T ,pog_3[0:3])

    pvfront_3_1 = np.subtract(pbg_3,pag_3) 
    pvfront_3_2 = np.subtract(pbg_3,pcg_3)
    norm_3_5 = np.cross(pvfront_3_1[0:3], pvfront_3_2[0:3]) 
    d_3_5 = np.matmul(norm_3_5.T ,pag_3[0:3])
    ##################################################################
    ##################################################################
    #Matrix Multiplication of 4th pyramid
    pog_4 = np.matmul(T_AL ,po_4)
    pag_4 = np.matmul(T_AL ,pa_4)
    pbg_4 = np.matmul(T_AL ,pb_4)
    pcg_4 = np.matmul(T_AL ,pc_4)
    pdg_4 = np.matmul(T_AL ,pd_4)
    psg_4 = np.matmul(T_AL ,ps_4)
    #Matrix Multiplication of fourth pyramid's top plane normal and d value
    pvtop_4_1 = np.subtract(pog_4,pag_4)
    pvtop_4_2 = np.subtract(pog_4,pbg_4)
    norm_4_1 = np.cross(pvtop_4_1[0:3], pvtop_4_2[0:3]) 
    d_4_1 = np.matmul(norm_4_1.T ,pog_4[0:3])
    
    pvside_4_1 = np.subtract(pog_4,pcg_4)
    pvside_4_2 = np.subtract(pog_4,pbg_4)
    norm_4_2 = np.cross(pvside_4_1[0:3], pvside_4_2[0:3]) 
    d_4_2 = np.matmul(norm_4_2.T ,pog_4[0:3])

    pvbot_4_1 = np.subtract(pog_4,pcg_4)
    pvbot_4_2 = np.subtract(pog_4,pdg_4) 
    norm_4_3 = np.cross(pvbot_4_1[0:3], pvbot_4_2[0:3]) 
    d_4_3 = np.matmul(norm_4_3.T ,pog_4[0:3])

    pvside_4_11 = np.subtract(pog_4,pag_4)
    pvside_4_22 = np.subtract(pog_4,pdg_4) 
    norm_4_4 = np.cross(pvside_4_11[0:3], pvside_4_22[0:3]) 
    d_4_4 = np.matmul(norm_4_4.T ,pog_4[0:3])

    pvfront_4_1 = np.subtract(pbg_4,pag_4) 
    pvfront_4_2 = np.subtract(pbg_4,pcg_4)
    norm_4_5 = np.cross(pvfront_4_1[0:3], pvfront_4_2[0:3]) 
    d_4_5 = np.matmul(norm_4_5.T ,pag_4[0:3])
    ##################################################################
    ##Normal direction verification for sample point in pyramid 1
    num_s = norm_1[0]*psg_1[0] + norm_1[1]*psg_1[1] + norm_1[2]*psg_1[2] - d_1
    den_s = math.sqrt(norm_1[0]**2+norm_1[1]**2+norm_1[2]**2)
    D_1_2_s[0] = num_s/den_s
    num_2_s = norm_2[0]*psg_1[0] + norm_2[1]*psg_1[1] + norm_2[2]*psg_1[2] - d_2
    den_2_s = math.sqrt(norm_2[0]**2+norm_2[1]**2+norm_2[2]**2)
    D_1_2_s[1]= num_2_s/den_2_s
    num_3_s = norm_3[0]*psg_1[0] + norm_3[1]*psg_1[1] + norm_3[2]*psg_1[2] - d_3
    den_3_s = math.sqrt(norm_3[0]**2+norm_3[1]**2+norm_3[2]**2)
    D_1_2_s[2] = num_3_s/den_3_s
    num_4_s = norm_4[0]*psg_1[0] + norm_4[1]*psg_1[1] + norm_4[2]*psg_1[2] - d_4
    den_4_s = math.sqrt(norm_4[0]**2+norm_4[1]**2+norm_4[2]**2)
    D_1_2_s[3] = num_4_s/den_4_s
    num_5_s = norm_5[0]*psg_1[0] + norm_5[1]*psg_1[1] + norm_5[2]*psg_1[2] - d_5
    den_5_s = math.sqrt(norm_5[0]**2+norm_5[1]**2+norm_5[2]**2)
    D_1_2_s[4] = num_5_s/den_5_s
    ##Normal direction verification for sample point in pyramid 2
    num_2_s = norm_2_1[0]*psg_2[0] + norm_2_1[1]*psg_2[1] + norm_2_1[2]*psg_2[2] - d_2_1
    den_2_s = math.sqrt(norm_2_1[0]**2+norm_2_1[1]**2+norm_2_1[2]**2)
    D_2_2_s[0] = num_2_s/den_2_s
    num_2_2_s = norm_2_2[0]*psg_2[0] + norm_2_2[1]*psg_2[1] + norm_2_2[2]*psg_2[2] - d_2_2
    den_2_2_s = math.sqrt(norm_2_2[0]**2+norm_2_2[1]**2+norm_2_2[2]**2)
    D_2_2_s[1]= num_2_2_s/den_2_2_s
    num_2_3_s = norm_2_3[0]*psg_2[0] + norm_2_3[1]*psg_2[1] + norm_2_3[2]*psg_2[2] - d_2_3
    den_2_3_s = math.sqrt(norm_2_3[0]**2+norm_2_3[1]**2+norm_2_3[2]**2)
    D_2_2_s[2] = num_2_3_s/den_2_3_s
    num_2_4_s = norm_2_4[0]*psg_2[0] + norm_2_4[1]*psg_2[1] + norm_2_4[2]*psg_2[2] - d_2_4
    den_2_4_s = math.sqrt(norm_2_4[0]**2+norm_2_4[1]**2+norm_2_4[2]**2)
    D_2_2_s[3] = num_2_4_s/den_2_4_s
    num_2_5_s = norm_2_5[0]*psg_2[0] + norm_2_5[1]*psg_2[1] + norm_2_5[2]*psg_2[2] - d_2_5
    den_2_5_s = math.sqrt(norm_2_5[0]**2+norm_2_5[1]**2+norm_2_5[2]**2)
    D_2_2_s[4] = num_2_5_s/den_2_5_s
    ##Normal direction verification for sample point in pyramid 3
    num_3_s = norm_3_1[0]*psg_3[0] + norm_3_1[1]*psg_3[1] + norm_3_1[2]*psg_3[2] - d_3_1
    den_3_s = math.sqrt(norm_3_1[0]**2+norm_3_1[1]**2+norm_3_1[2]**2)
    D_3_2_s[0] = num_3_s/den_3_s
    num_3_2_s = norm_3_2[0]*psg_3[0] + norm_3_2[1]*psg_3[1] + norm_3_2[2]*psg_3[2] - d_3_2
    den_3_2_s = math.sqrt(norm_3_2[0]**2+norm_3_2[1]**2+norm_3_2[2]**2)
    D_3_2_s[1]= num_3_2_s/den_3_2_s
    num_3_3_s = norm_3_3[0]*psg_3[0] + norm_3_3[1]*psg_3[1] + norm_3_3[2]*psg_3[2] - d_3_3
    den_3_3_s = math.sqrt(norm_3_3[0]**2+norm_3_3[1]**2+norm_3_3[2]**2)
    D_3_2_s[2] = num_3_3_s/den_3_3_s
    num_3_4_s = norm_3_4[0]*psg_3[0] + norm_3_4[1]*psg_3[1] + norm_3_4[2]*psg_3[2] - d_3_4
    den_3_4_s = math.sqrt(norm_3_4[0]**2+norm_3_4[1]**2+norm_3_4[2]**2)
    D_3_2_s[3] = num_3_4_s/den_3_4_s
    num_3_5_s = norm_3_5[0]*psg_3[0] + norm_3_5[1]*psg_3[1] + norm_3_5[2]*psg_3[2] - d_3_5
    den_3_5_s = math.sqrt(norm_3_5[0]**2+norm_3_5[1]**2+norm_3_5[2]**2)
    D_3_2_s[4] = num_3_5_s/den_3_5_s
    ##Normal direction verification for sample point in pyramid 4
    num_4_s = norm_4_1[0]*psg_4[0] + norm_4_1[1]*psg_4[1] + norm_4_1[2]*psg_4[2] - d_4_1
    den_4_s = math.sqrt(norm_4_1[0]**2+norm_4_1[1]**2+norm_4_1[2]**2)
    D_4_2_s[0] = num_4_s/den_4_s
    num_4_2_s = norm_4_2[0]*psg_4[0] + norm_4_2[1]*psg_4[1] + norm_4_2[2]*psg_4[2] - d_4_2
    den_4_2_s = math.sqrt(norm_4_2[0]**2+norm_4_2[1]**2+norm_4_2[2]**2)
    D_4_2_s[1]= num_4_2_s/den_4_2_s
    num_4_3_s = norm_4_3[0]*psg_4[0] + norm_4_3[1]*psg_4[1] + norm_4_3[2]*psg_4[2] - d_4_3
    den_4_3_s = math.sqrt(norm_4_3[0]**2+norm_4_3[1]**2+norm_4_3[2]**2)
    D_4_2_s[2] = num_4_3_s/den_4_3_s
    num_4_4_s = norm_4_4[0]*psg_4[0] + norm_4_4[1]*psg_4[1] + norm_4_4[2]*psg_4[2] - d_4_4
    den_4_4_s = math.sqrt(norm_4_4[0]**2+norm_4_4[1]**2+norm_4_4[2]**2)
    D_4_2_s[3] = num_4_4_s/den_4_4_s
    num_4_5_s = norm_4_5[0]*psg_4[0] + norm_4_5[1]*psg_4[1] + norm_4_5[2]*psg_4[2]- d_4_5
    den_4_5_s = math.sqrt(norm_4_5[0]**2+norm_4_5[1]**2+norm_4_5[2]**2)
    D_4_2_s[4] = num_4_5_s/den_4_5_s
    ##################################################################
    ##Begining of DRONE 2,3,4 Inclusion Test using a for Loop
    ##Distance from Pyramids' points comparison dist_p1_d2 is distance of pyramid 1 points from d2, drone 2
    # make this into a loop for each neighboring agent
    for a in range(agents-1):#looping with range method ex: rang(2) -> output is 0,1
        #pub_incl = rospy.Publisher('/%s_%d/incl_test'("guidance", a+2), Int32, queue_size=40)#not sure if this will work, still need to check
        pyr_incl = np.zeros(4)
        dist_p1_d2[0] = math.sqrt((pos_list[a,0] - pag_1[0] )**2 + (pos_list[a,1] - pag_1[1])**2 +  ( pos_list[a,2] - pag_1[2])**2 )
        dist_p1_d2[1] = math.sqrt((pos_list[a,0] - pbg_1[0] )**2 + (pos_list[a,1] - pbg_1[1])**2 + ( pos_list[a,2] - pbg_1[2])**2)
        dist_p1_d2[2] = math.sqrt((pos_list[a,0] - pcg_1[0] )**2 + (pos_list[a,1] - pcg_1[1])**2 + ( pos_list[a,2] - pcg_1[2])**2)
        dist_p1_d2[3] = math.sqrt((pos_list[a,0] - pdg_1[0] )**2 + (pos_list[a,1] - pdg_1[1])**2 + ( pos_list[a,2] - pdg_1[2])**2)
        dist_comp[0] = min(dist_p1_d2)
        ##Distance from Pyramids' points comparison dist_p1_d2 is distance of pyramid 2 points from d2, drone 2
        dist_p2_d2[0] = math.sqrt((pos_list[a,0] - pag_2[0] )**2 + (pos_list[a,1] - pag_2[1])**2 + ( pos_list[a,2] - pag_2[2])**2)
        dist_p2_d2[1] = math.sqrt((pos_list[a,0] - pbg_2[0] )**2 + (pos_list[a,1] - pbg_2[1])**2 + ( pos_list[a,2] - pbg_2[2])**2)
        dist_p2_d2[2] = math.sqrt((pos_list[a,0] - pcg_2[0] )**2 + (pos_list[a,1] - pcg_2[1])**2 + ( pos_list[a,2] - pcg_2[2])**2)
        dist_p2_d2[3] = math.sqrt((pos_list[a,0] - pdg_2[0] )**2 + (pos_list[a,1] - pdg_2[1])**2 + ( pos_list[a,2] - pdg_2[2])**2)
        dist_comp[1] = min(dist_p2_d2)
        ##Distance from Pyramids' points comparison dist_p1_d2 is distance of pyramid 3 points from d2, drone 2
        dist_p3_d2[0] = math.sqrt((pos_list[a,0] - pag_3[0] )**2 + (pos_list[a,1]- pag_3[1])**2 + ( pos_list[a,2] - pag_3[2])**2)
        dist_p3_d2[1] = math.sqrt((pos_list[a,0] - pbg_3[0] )**2 + (pos_list[a,1] - pbg_3[1])**2 + ( pos_list[a,2] - pbg_3[2])**2)
        dist_p3_d2[2] = math.sqrt((pos_list[a,0] - pcg_3[0] )**2 + (pos_list[a,1] - pcg_3[1])**2 + ( pos_list[a,2] - pcg_3[2])**2)
        dist_p3_d2[3] = math.sqrt((pos_list[a,0] - pdg_3[0] )**2 + (pos_list[a,1] - pdg_3[1])**2 + ( pos_list[a,2] - pdg_3[2])**2)
        dist_comp[2] = min(dist_p3_d2)
        ##Distance from Pyramids' points comparison dist_p1_d2 is distance of pyramid 4 points from d2, drone 2
        dist_p4_d2[0] = math.sqrt((pos_list[a,0] - pag_4[0] )**2 + (pos_list[a,1] - pag_4[1])**2 + ( pos_list[a,2] - pag_4[2])**2)
        dist_p4_d2[1] = math.sqrt((pos_list[a,0] - pbg_4[0] )**2 + (pos_list[a,1] - pbg_4[1])**2 + ( pos_list[a,2] - pbg_4[2])**2)
        dist_p4_d2[2] = math.sqrt((pos_list[a,0] - pcg_4[0] )**2 + (pos_list[a,1] - pcg_4[1])**2 + ( pos_list[a,2] - pcg_4[2])**2)
        dist_p4_d2[3] = math.sqrt((pos_list[a,0] - pdg_4[0] )**2 + (pos_list[a,1] - pdg_4[1])**2 + ( pos_list[a,2] - pdg_4[2])**2)
        dist_comp[3] = min(dist_p4_d2)
        ##Decide which pyramid is Drone 2 closest to
        if (pos_list[a,0] != 0 and pos_list[a,1] != 0 and pos_list[a,2] != 0): #To check if drone even exists in the x-y-z- grid because x=y=z=0 is not possible
            minni= min(dist_comp)
            index_dist_comp = dist_comp.tolist().index(minni)#WHAT HAPPENS IF THE DISTANCE IS EQUAL FROM TWO PYRAMIDS
            pyramid_no =  index_dist_comp + 1
            ##for drone 2 and 1st pyramid inclusion test
            #if pyramid_no == 1:
            num = norm_1[0]*pos_list[a,0] + norm_1[1]*pos_list[a,1] + norm_1[2]*pos_list[a,2] - d_1
            den = math.sqrt(norm_1[0]**2+norm_1[1]**2+norm_1[2]**2)
            D_1_2[0] = num/den
            num_2 = norm_2[0]*pos_list[a,0] + norm_2[1]*pos_list[a,1] + norm_2[2]*pos_list[a,2] - d_2
            den_2 = math.sqrt(norm_2[0]**2+norm_2[1]**2+norm_2[2]**2)
            D_1_2[1]= num_2/den_2
            num_3 = norm_3[0]*pos_list[a,0] + norm_3[1]*pos_list[a,1] + norm_3[2]*pos_list[a,2] - d_3
            den_3 = math.sqrt(norm_3[0]**2+norm_3[1]**2+norm_3[2]**2)
            D_1_2[2] = num_3/den_3
            num_4 = norm_4[0]*pos_list[a,0] + norm_4[1]*pos_list[a,1] + norm_4[2]*pos_list[a,2] - d_4
            den_4 = math.sqrt(norm_4[0]**2+norm_4[1]**2+norm_4[2]**2)
            D_1_2[3] = num_4/den_4
            num_5 = norm_5[0]*pos_list[a,0] + norm_5[1]*pos_list[a,1] + norm_5[2]*pos_list[a,2] - d_5
            den_5 = math.sqrt(norm_5[0]**2+norm_5[1]**2+norm_5[2]**2)
            D_1_2[4] = num_5/den_5
            #print(num_5)
            #print(den_5)
            for i in range(len(D_1_2)):
                    if D_1_2[i]/abs(D_1_2[i]) == D_1_2_s[i]/abs(D_1_2_s[i]):
                        #print(D_1_2[i])
                        #print( D_1_2_s[i])
                        bool_data[i] = 1
                    else:
                        #print(D_1_2[i])
                        #print( D_1_2_s[i])
                        bool_data[i] = 0
                # Calculate Euclidean distance of Drone Position with respect to sample point psg_1 for pyramid 1 for x-y coordinate system as Pozyx is running on 2D algorithm
            euc_dist = math.sqrt((pos_list[a,0] - psg_1[0] )**2 + (pos_list[a,1]- psg_1[1])**2 + (pos_list[a,2]- psg_1[2])**2) 
            if min(bool_data) == 0:
                    #print('out_1')
                    #print(a+2)
                    #adj[a+1] = 0 #This will remain zero, which means nhbd agent is not in FOV
                    pyr_incl[0] = 0
                    #pub_incl.publish(0)
            else:
                    #print('in_1')
                    #print(a+2)
                    pyr_incl[0] =  euc_dist #it will be euc distance instead of 0
                    #pub_incl.publish(1)
                
            
            #if pyramid_no == 2:
            num_2 = norm_2_1[0]*pos_list[a,0] + norm_2_1[1]*pos_list[a,1] + norm_2_1[2]*pos_list[a,2] - d_2_1
            den_2 = math.sqrt(norm_2_1[0]**2+norm_2_1[1]**2+norm_2_1[2]**2)
            D_2_2[0] = num_2/den_2
            num_2_2 = norm_2_2[0]*pos_list[a,0] + norm_2_2[1]*pos_list[a,1] + norm_2_2[2]*pos_list[a,2] - d_2_2
            den_2_2 = math.sqrt(norm_2_2[0]**2+norm_2_2[1]**2+norm_2_2[2]**2)
            D_2_2[1]= num_2_2/den_2_2
            num_2_3 = norm_2_3[0]*pos_list[a,0] + norm_2_3[1]*pos_list[a,1] + norm_2_3[2]*pos_list[a,2] - d_2_3
            den_2_3 = math.sqrt(norm_2_3[0]**2+norm_2_3[1]**2+norm_2_3[2]**2)
            D_2_2[2] = num_2_3/den_2_3
            num_2_4 = norm_2_4[0]*pos_list[a,0] + norm_2_4[1]*pos_list[a,1] + norm_2_4[2]*pos_list[a,2] - d_2_4
            den_2_4 = math.sqrt(norm_2_4[0]**2+norm_2_4[1]**2+norm_2_4[2]**2)
            D_2_2[3] = num_2_4/den_2_4
            num_2_5 = norm_2_5[0]*pos_list[a,0] + norm_2_5[1]*pos_list[a,1] + norm_2_5[2]*pos_list[a,2] - d_2_5
            den_2_5 = math.sqrt(norm_2_5[0]**2+norm_2_5[1]**2+norm_2_5[2]**2)
            D_2_2[4] = num_2_5/den_2_5
            for i in range(len(D_2_2)):
                    if D_2_2[i]/abs(D_2_2[i]) == D_2_2_s[i]/abs(D_2_2_s[i]):
                        bool_data[i] = 1
                        
                    else:
                        bool_data[i] = 0
            euc_dist = math.sqrt((pos_list[a,0] - psg_2[0] )**2 + (pos_list[a,1]- psg_2[1])**2 + (pos_list[a,2]- psg_2[2])**2 ) 
            if min(bool_data) == 0:
                    #print('out_2')
                    #print(a+2)
                    pyr_incl[1]  = 0
                    #print(adj)
                    #pub_incl.publish(0)
            else:
                    #print('in_2')
                    #print(a+2)
                    pyr_incl[1]  = euc_dist #it will be euc distance instead of 0
                    #pub_incl.publish(1)
                
            #if pyramid_no == 3:
            num_3 = norm_3_1[0]*pos_list[a,0] + norm_3_1[1]*pos_list[a,1] + norm_3_1[2]*pos_list[a,2] - d_3_1
            den_3 = math.sqrt(norm_3_1[0]**2+norm_3_1[1]**2+norm_3_1[2]**2)
            D_3_2[0] = num_3/den_3
            num_3_2 = norm_3_2[0]*pos_list[a,0] + norm_3_2[1]*pos_list[a,1] + norm_3_2[2]*pos_list[a,2] - d_3_2
            den_3_2 = math.sqrt(norm_3_2[0]**2+norm_3_2[1]**2+norm_3_2[2]**2)
            D_3_2[1]= num_3_2/den_3_2
            num_3_3 = norm_3_3[0]*pos_list[a,0] + norm_3_3[1]*pos_list[a,1] + norm_3_3[2]*pos_list[a,2] - d_3_3
            den_3_3 = math.sqrt(norm_3_3[0]**2+norm_3_3[1]**2+norm_3_3[2]**2)
            D_3_2[2] = num_3_3/den_3_3
            num_3_4 = norm_3_4[0]*pos_list[a,0] + norm_3_4[1]*pos_list[a,1] + norm_3_4[2]*pos_list[a,2] - d_3_4
            den_3_4 = math.sqrt(norm_3_4[0]**2+norm_3_4[1]**2+norm_3_4[2]**2)
            D_3_2[3] = num_3_4/den_3_4
            num_3_5 = norm_3_5[0]*pos_list[a,0] + norm_3_5[1]*pos_list[a,1] + norm_3_5[2]*pos_list[a,2] - d_3_5
            den_3_5 = math.sqrt(norm_3_5[0]**2+norm_3_5[1]**2+norm_3_5[2]**2)
            D_3_2[4] = num_3_5/den_3_5
            for i in range(len(D_3_2)):
                    if D_3_2[i]/abs(D_3_2[i]) == D_3_2_s[i]/abs(D_3_2_s[i]):
                        bool_data[i] = 1
                        
                    else:
                        bool_data[i] = 0
            euc_dist = math.sqrt((pos_list[a,0] - psg_3[0] )**2 + (pos_list[a,1]- psg_3[1])**2 + (pos_list[a,2]- psg_3[2])**2) 
            if min(bool_data) == 0:
                    #print('out_3')
                    #print (a+2)
                    pyr_incl[2] = 0
                    #pub_incl.publish(0)
            else:
                    #print('in_3')
                    #print(a+2)
                    pyr_incl[2] = euc_dist #it will be euc distance instead of 0
                    #pub_incl.publish(1)
                
            #if pyramid_no == 4:
            num_4 = norm_4_1[0]*pos_list[a,0] + norm_4_1[1]*pos_list[a,1] + norm_4_1[2]*pos_list[a,2] - d_4_1
            den_4 = math.sqrt(norm_4_1[0]**2+norm_4_1[1]**2+norm_4_1[2]**2)
            D_4_2[0] = num_4/den_4
            num_4_2 = norm_4_2[0]*pos_list[a,0] + norm_4_2[1]*pos_list[a,1] + norm_4_2[2]*pos_list[a,2] - d_4_2
            den_4_2 = math.sqrt(norm_4_2[0]**2+norm_4_2[1]**2+norm_4_2[2]**2)
            D_4_2[1]= num_4_2/den_4_2
            num_4_3 = norm_4_3[0]*pos_list[a,0] + norm_4_3[1]*pos_list[a,1] + norm_4_3[2]*pos_list[a,2] - d_4_3
            den_4_3 = math.sqrt(norm_4_3[0]**2+norm_4_3[1]**2+norm_4_3[2]**2)
            D_4_2[2] = num_4_3/den_4_3
            num_4_4 = norm_4_4[0]*pos_list[a,0] + norm_4_4[1]*pos_list[a,1] + norm_4_4[2]*pos_list[a,2] - d_4_4
            den_4_4 = math.sqrt(norm_4_4[0]**2+norm_4_4[1]**2+norm_4_4[2]**2)
            D_4_2[3] = num_4_4/den_4_4
            num_4_5 = norm_4_5[0]*pos_list[a,0] + norm_4_5[1]*pos_list[a,1] + norm_4_5[2]*pos_list[a,2] - d_4_5
            den_4_5 = math.sqrt(norm_4_5[0]**2+norm_4_5[1]**2+norm_4_5[2]**2)
            D_4_2[4] = num_4_5/den_4_5
            for i in range(len(D_4_2)):
                    if D_4_2[i]/abs(D_4_2[i]) == D_4_2_s[i]/abs(D_4_2_s[i]):
                        bool_data[i] = 1
                    else:
                        bool_data[i] = 0
            euc_dist = math.sqrt((pos_list[a,0] - psg_4[0] )**2 + (pos_list[a,1]- psg_4[1])**2 + (pos_list[a,2]- psg_4[2])**2 ) 
            if min(bool_data) == 0:
                    #print('out_4')
                    #print(a+2)
                    pyr_incl[3] = 0  
            else:
                    #print('in_4')
                    #print(a+2)
                    pyr_incl[3] = euc_dist #it will be euc distance instead of 0
            if a==1:
               adj[0] = max(pyr_incl)
            else :
               adj[a+1] = max(pyr_incl)

                
        ##END of DRONE  Inclusion Test
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    pub = rospy.Publisher('/firefly_3/fov_adj', numpy_msg(Floats),queue_size=10)
    rospy.init_node('fov3', anonymous=True)
    
    #Subscribing to Experimental Topics
    rospy.Subscriber("/firefly_2/ground_truth/pose", Pose, pos_2)
    rospy.Subscriber("/firefly_1/ground_truth/pose", Pose, pos_3)
    rospy.Subscriber("/firefly_4/ground_truth/pose", Pose, pos_4)
    rospy.Subscriber("/firefly_3/ground_truth/pose", Pose, fov) 
    #########################################################################################
    

    #############################################################################################
    rate = rospy.Rate(10) # 10hz, may need this to be 100HZ
    while not rospy.is_shutdown():
        
        #a = np.array([2.10,3.10,2.14,2.34,2.34,3.14],dtype=np.float32)
        pub.publish(adj)
        rate.sleep()
        
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()