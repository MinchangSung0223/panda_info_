import sys
import os
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import pybullet as p
import signal
import time
import sys

def signal_handler(signal, frame):
  # your code here
  sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
from pybullet import getQuaternionFromEuler,getEulerFromQuaternion,getMatrixFromQuaternion,invertTransform
import numpy as np
np.set_printoptions(formatter={'float_kind': lambda x: "{0:0.6f}".format(x)})
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
def getHomogeneousFormFromPose(eef_pose):
	#end-effector quaternion orietation
	eef_quat_ori = np.array([eef_pose.orientation.x,eef_pose.orientation.y,eef_pose.orientation.z,eef_pose.orientation.w]) 
	eef_R = getMatrixFromQuaternion(eef_quat_ori);
	Tbe = np.eye(4)
	Tbe[0:3,0:3] = np.reshape(eef_R,[3,3])
	Tbe[0,3] = eef_pose.position.x
	Tbe[1,3] = eef_pose.position.y
	Tbe[2,3] = eef_pose.position.z
	R = np.reshape(p.getMatrixFromQuaternion(p.getQuaternionFromEuler([0,0,-pi/4])),[3,3])
	Temp = np.eye(4)
	Temp[0:3,0:3] = R
	Tbe = np.matmul(Tbe,Temp)
	return Tbe
def getMatrixfromIntrinsicParameter():
	try:
		print("------------------------------")
		print("\t load intrinsic params")
		K = np.loadtxt("intrinsic.txt")
	except:
		print("------------------------------")
		print(" intrinsic Matrix K is ")
		print(" fx  skew cx ")
		print("  0   fy  cy ")
		print("  0   0    1")
		print("------------------------------")

		fx = input("focal length x (fx) : ")
		fy = input("focal length y (fy) : ")
		cx = input("optical center x (cx) : ")
		cy = input("optical center y (cy) : ")
		skew = input("skew (skew) : ")
		K = np.eye(3)
		K[0,0] = fx;
		K[0,1] = skew;
		K[1,1] = fy;
		K[0,2] = cx;
		K[1,2] = cy;
	print("------------------------------")
	print("input intrinsic Matrix K is ")
	print(K)
	print("------------------------------")
	return K

def getEndEffectorToCamPose():
	print("------------------------------")
	print("\t load calibration_result params")
	Teb = np.eye(4)
	Tcb = np.loadtxt("calibration_result.txt")
	Tcb[2,2] =-1*Tcb[2,2] 
	Rcb = Tcb[0:3,0:3];
	Rbe = np.eye(3)
	Rbe[1,1] = -1
	Rbe[2,2] = -1
	Rce = np.matmul(Rcb,Rbe)
	Teb[0:3,0:3] = np.transpose(Rce)
	Teb[0,3] = Tcb[0,3]
	Teb[1,3] = Tcb[1,3]
	Teb[2,3] = Tcb[2,3]
	print(Teb)
	return Teb


def convert_depth_frame_to_pointcloud(depth_image,K):
	camera_intrinsics =K
	fx = K[0,0]
	fy = K[1,1]
	cx = K[0,2]
	cy = K[1,2]

	[height, width] = depth_image.shape
	nx = np.linspace(0, width-1, width)
	ny = np.linspace(0, height-1, height)
	u, v = np.meshgrid(nx, ny)
	x = (u.flatten() - cx)/fx
	y = (v.flatten() - cy)/fy

	z = depth_image.flatten() / 1000.0;
	x = np.multiply(x,z)
	y = np.multiply(y,z)

	x = x[np.nonzero(z)]
	y = y[np.nonzero(z)]
	z = z[np.nonzero(z)]
        ptcloud = np.transpose(np.array([x,y,z],dtype=np.double))
	return ptcloud

def point_to_baseframe(ptcloud,Tbc):
	# --------Change World Frame-------------
	Tcb = np.linalg.inv(Tbc) 
	ptcloud = np.matmul(Tcb,np.transpose(np.array([ptcloud[0],ptcloud[1],ptcloud[2],1])))
	return ptcloud

def convert_imagepoint_to_worldpoint(u,v,z,K):
	# --------Camera Frame-------------
	# u  width pixel
	# v  height pixel

        # z [m]
	camera_intrinsics =K
	fx = K[0,0]
	fy = K[1,1]
	cx = K[0,2]
	cy = K[1,2]
	u = np.array(u,np.double)
	v = np.array(v,np.double)
	z = np.array(z,np.double)

	x = (u- cx)/fx
	y = (v- cy)/fy
	z = z;
	x = np.multiply(x,z)
	y = np.multiply(y,z)
        ptcloud = np.transpose(np.array([x,y,z],dtype=np.double))
	return ptcloud
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_test',anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
planning_frame = move_group.get_planning_frame()
print(planning_frame)
#end-effector-pose
os.system('clear')
while(1):

	eef_pose = move_group.get_current_pose().pose 
	Tbe = getHomogeneousFormFromPose(eef_pose) #base to eef
	K = getMatrixfromIntrinsicParameter() #intrinsic

	Tec = getEndEffectorToCamPose() #base to cam
	Tbc =np.matmul(Tbe,Tec)

	print("-------------Tbe--------------")
	print(Tbe)
	print("-------------Tec--------------")
	print(Tec)
	print("-------------Tbc--------------")
	print(Tbc)

	cx = K[0,2]
	cy = K[1,2]
	print("-------------cam_xyz--------------")
	u = cx
	v = cy
	z = 0.590 # [m]
	print("Image Points : "+str(u)+","+str(v))
	cam_xyz = 	convert_imagepoint_to_worldpoint(u,v,z,K)
	print("camera frame World Points : "+str(cam_xyz))
	print("-------------world_xyz--------------")
	world_xyz = point_to_baseframe(cam_xyz,Tbc)
	print("base frame World Points : "+str(world_xyz))
	time.sleep(0.01)






