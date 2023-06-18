#!/usr/bin/env python2

# Robot - MoveIt resources

import os
import sys
import copy
import rospy
import moveit_commander
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from math import pi
import tkinter as tk
from tkinter import ttk
from tf.transformations import quaternion_from_euler
# Gripper - TCP/IP socket


import time
import signal
#from RobotiqHand import RobotiqHand

HOST = "ur"
PORT = 54321
cont = True

# ROS messages

from std_msgs.msg import Int8
from sensor_msgs.msg import Joy

AXIS_MIN = -1
AXIS_MAX = 1

# Mapeo de los ejes del joystick a las articulaciones del robot UR5
JOINT_MAPPING = {
    0: 'shoulder_pan_joint',
    1: 'shoulder_lift_joint',
    2: 'elbow_joint',
    3: 'wrist_1_joint',
    4: 'wrist_2_joint',
    5: 'wrist_3_joint'
}

# Callbacks
class manipulator_control:
	def mode_callback(data):

		global mode

		mode = data.data

		print("Received mode command: ", mode)

	# Funcion para salir del programa
	def quit(self):
		self.root.quit()
		self.root.destroy()
		sys.exit()

	def joy_callback(self, data):

		global lr, fb, ud, teleopFinish

		lr = data.axes[0]
		fb = data.axes[1]
		ud = data.axes[4]
		if (data.buttons[1] != 0):
			teleopFinish = 1
		#print("lr = " + str(lr) + "\nfb: " + str(fb) + "\nud: " + str(ud) + "\ntpFinish: " + str(teleopFinish))
		# Escalar los valores de los joysticks a velocidades de joint
		"""joint_velocities = [msg.axes[1] * 0.5, msg.axes[0] * 0.5, msg.axes[3] * 0.5, msg.axes[4] * 0.5,
							msg.axes[2] * 0.5, msg.axes[5] * 0.5]
		print(joint_velocities)
		print("hola")

		# Calcular los nuevos valores de los joints
		new_joint_values = [self.current_joint_values[i] + joint_velocities[i] for i in range(len(self.joint_names))]

		# Crear un nuevo mensaje de seguimiento de trayectoria
		traj_goal = FollowJointTrajectoryAction()
		traj_goal.goal = FollowJointTrajectoryGoal()
		traj_goal.goal.trajectory.joint_names = self.joint_names
		traj_goal.goal.trajectory.points.append(JointTrajectoryPoint())
		traj_goal.goal.trajectory.points[0].positions = new_joint_values
		traj_goal.goal.trajectory.points[0].time_from_start = rospy.Duration(0.1)

		# Publicar el comando de movimiento del robot
		self.command_publisher.publish(traj_goal)

		# Actualizar los valores actuales de los joints
		self.current_joint_values = new_joint_values"""

	def callback(self, msg):
		global joint_values
		joint_values = msg.position

	# Gripper - Aux functions

	"""def create(self):
		# Create hand
		hand = RobotiqHand()
		# Connect to hand
		hand.connect(HOST, PORT)
		# Activate hand
		hand.reset()
		hand.activate()
		result = hand.wait_activate_complete()

		return hand"""

	"""def capture(self, hand):
		try:
			hand.move(255, 0, 20)
			(status, position, force) = hand.wait_move_complete()	    
			position_mm = hand.get_position_mm(position)
			force_mA = hand.get_force_mA(force)
	
			if status == 0:
				print('no object captured: position = {:.1f}mm, force = {:.1f}mA '.format(position_mm, force_mA))
			elif status == 1:
				print('object captured while closing: position = {:.1f}mm, force = {:.1f}mA '.format(position_mm, force_mA))
				time.sleep(2)
			elif status == 2:
				print('object captured while opening: position = {:.1f}mm, force = {:.1f}mA '.format(position_mm, force_mA))
			else:
				print('failed')
	
			return True
	
		except:
			return False"""

	"""def keep(self, hand):

		hand.move(255, 0, 20)
		(status, position, force) = hand.wait_move_complete()
		position_mm = hand.get_position_mm(position)
		force_mA = hand.get_force_mA(force)"""

	"""def release(self, hand):
	
		hand.move(0, 255, 0)
		(status, position, force) = hand.wait_move_complete()
		position_mm = hand.get_position_mm(position)
		force_mA = hand.get_force_mA(force)
		print('position = {:.1f}mm, force = {:.1f}mA '.format(position_mm, force_mA))
	
		return True"""

	"""def destroy(self, hand):

		hand.disconnect()

		return True"""

	def home_position(self):
		print("Home position")

		joint_goal = self.group.get_current_joint_values()

		joint_goal[0] = 0.2440
		joint_goal[1] = -2.7967
		joint_goal[2] = 2.7993
		joint_goal[3] = 1.8896
		joint_goal[4] = 4.7162
		joint_goal[5] = 1.4695

		self.group.go(joint_goal, wait=True)

		self.group.stop()

	def capture_position(self):
		print("Capture position")

		joint_goal = self.group.get_current_joint_values()

		joint_goal[0] = 0
		joint_goal[1] = -1.2564
		joint_goal[2] = 2.3797
		joint_goal[3] = 3.5299
		joint_goal[4] = 4.7014
		joint_goal[5] = 1.3583

		self.group.go(joint_goal, wait=True)

		self.group.stop()

	def release_position_left(self):
		print("Release position")

		joint_goal = self.group.get_current_joint_values()
		print("1st step")
		joint_goal[0] = 1.62
		joint_goal[1] = -1.988
		joint_goal[2] = 2.180
		joint_goal[3] = 4.523
		joint_goal[4] = 4.661
		joint_goal[5] = 0.827
		self.group.go(joint_goal, wait=True)
		#self.group.stop()

		joint_goal = self.group.get_current_joint_values()
		print("2nd step")
		joint_goal[0] = 1.62
		joint_goal[1] = -2.36
		joint_goal[2] = 2.22
		joint_goal[3] = 4.88
		joint_goal[4] = 4.76
		joint_goal[5] = 1.054
		self.group.go(joint_goal, wait=True)
		#self.group.stop()

		joint_goal = self.group.get_current_joint_values()
		print("3rd step")
		joint_goal[0] = 2.61
		joint_goal[1] = -2.36
		joint_goal[2] = 2.22
		joint_goal[3] = 4.88
		joint_goal[4] = 4.76
		joint_goal[5] = 1.054
		self.group.go(joint_goal, wait=True)
		self.group.stop()

	def release_position_right(self):
		print("Release position")

		joint_goal = self.group.get_current_joint_values()
		print("1st step")
		joint_goal[0] = -1.57
		joint_goal[1] = -1.988
		joint_goal[2] = 2.180
		joint_goal[3] = 4.523
		joint_goal[4] = 4.661
		joint_goal[5] = 0.827
		self.group.go(joint_goal, wait=True)
		#self.group.stop()

		joint_goal = self.group.get_current_joint_values()
		print("2nd step")
		joint_goal[0] = -1.57
		joint_goal[1] = -2.36
		joint_goal[2] = 2.22
		joint_goal[3] = 4.88
		joint_goal[4] = 4.76
		joint_goal[5] = 1.054
		self.group.go(joint_goal, wait=True)
		#self.group.stop()

		joint_goal = self.group.get_current_joint_values()
		print("3rd step")
		joint_goal[0] = -3.67
		joint_goal[1] = -2.36
		joint_goal[2] = 2.22
		joint_goal[3] = 4.88
		joint_goal[4] = 4.76
		joint_goal[5] = 1.054
		self.group.go(joint_goal, wait=True)
		self.group.stop()


	def teleoperation(self):
		print("Teleoperation")

		global teleopFinish

		while(teleopFinish == 0):
			waypoints = []
			print("lr = " + str(lr) + "\nfb: " + str(fb) + "\nud: " + str(ud) + "\ntpFinish: " + str(teleopFinish))
			wpose = self.group.get_current_pose().pose
			wpose.position.x += 0.05 * fb
			wpose.position.y += 0.05 * lr
			wpose.position.z += 0.05 * ud
			waypoints.append(copy.deepcopy(wpose))

			(plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)

			self.group.execute(plan, wait=True)
			time.sleep(1)

		teleopFinish = 0
		print("Teleoperation finished")

	def capture(self):
		print("Capture")

		#capture(hand)

	def teleoperation_object(self):
		# keep(hand)
		print("Teleoperation with object")
		waypoints = []

		wpose = self.group.get_current_pose().pose
		wpose.position.x -= 0.05 * fb
		wpose.position.y -= 0.05 * lr
		wpose.position.z += 0.05 * ud

		waypoints.append(copy.deepcopy(wpose))

		(plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)

		self.group.execute(plan, wait=True)

	def release(self):
		print("Release")

		#release(hand)

		mode = 0

	def ejecutar_modo_7(self):
		print("Modo 7 ejecutado")
		"""self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
							'wrist_2_joint', 'wrist_3_joint']
		self.current_joint_values = [0, 0, 0, 0, 0, 0]
		self.command_publisher = rospy.Publisher('/arm_controller/command', FollowJointTrajectoryAction,
												 queue_size=10)
		self.joy_subscriber = rospy.Subscriber('/joy', Joy, self.joy_callback)"""


	def create_fake_summit(self, scene, robot):
		objects = []

		# Crea un objeto box
		box_pose = geometry_msgs.msg.PoseStamped()
		box_pose.header.frame_id = robot.get_planning_frame()
		box_pose.pose.position.x = 0
		box_pose.pose.position.y = 0.2
		box_pose.pose.position.z = -0.196
		quaternion = quaternion_from_euler(0, 0, -1.57)
		box_pose.pose.orientation.x = quaternion[0]
		box_pose.pose.orientation.y = quaternion[1]
		box_pose.pose.orientation.z = quaternion[2]
		box_pose.pose.orientation.w = quaternion[3]
		box_name = "box"
		scene.add_box(box_name, box_pose, size=(0.72, 0.344, 0.392))
		objects.append(box_name)

	# Teleoperation function

	def __init__(self):

		global mode, lr, fb, ud, teleopFinish
		lr = 0
		fb = 0
		ud = 0
		teleopFinish = 0

		print("Initializing manipulator control node...")

		rospy.init_node('manipulator_control', anonymous=True)

		#rospy.Subscriber("mode", Int8, mode_callback)
		rospy.Subscriber("/joy", Joy, self.joy_callback)
		#rospy.Subscriber("joy", Twist, joy_callback)

		moveit_commander.roscpp_initialize(sys.argv)

		robot = moveit_commander.RobotCommander()

		print(robot.get_group_names())

		scene = moveit_commander.PlanningSceneInterface()

		"""objects = []

		# Crea un objeto box
		box_pose = geometry_msgs.msg.PoseStamped()
		box_pose.header.frame_id = robot.get_planning_frame()
		box_pose.pose.position.x = 0
		box_pose.pose.position.y = 0.2
		box_pose.pose.position.z = -0.196
		quaternion = quaternion_from_euler(0, 0, -1.57)
		box_pose.pose.orientation.x = quaternion[0]
		box_pose.pose.orientation.y = quaternion[1]
		box_pose.pose.orientation.z = quaternion[2]
		box_pose.pose.orientation.w = quaternion[3]
		box_name = "box"
		scene.add_box(box_name, box_pose, size=(0.72, 0.344, 0.392))
		objects.append(box_name)"""

		if len(sys.argv) == 2 and (sys.argv[1] == "S" or sys.argv[1] == "s"):
			self.create_fake_summit(scene, robot)

		self.group = moveit_commander.MoveGroupCommander("manipulator")

		#gripper = moveit_commander.MoveGroupCommander("endeffector")

		#display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

		#hand = create()
		self.root = tk.Tk()
		self.root.title("Control del brazo robotico")
		self.root.resizable(False, False)
		self.root.geometry("800x400+10+20")
		# Creamos un marco para los botones
		buttons = ttk.Frame(self.root, padding=30)
		buttons.pack()

		# Creamos un titulo centrado
		title = tk.Label(self.root, text="Escoge un modo de ejecucion", font=("Arial", 20))
		title.place(x=200, y=30)


		# Crear los botones

		modo_1_btn = tk.Button(self.root, text="Home position", command=self.home_position, width=20, height=3).place(x=100, y=100)
		modo_2_btn = tk.Button(self.root, text="Capture position", command=self.capture_position, width=20, height=3).place(x=300, y=100)
		modo_3_btn = tk.Button(self.root, text="Teleoperation", command=self.teleoperation, width=20, height=3).place(x=500, y=100)
		modo_4_btn = tk.Button(self.root, text="Capture", command=self.capture, width=20, height=3).place(x=100, y=175)
		modo_5_btn = tk.Button(self.root, text="Teleoperation with object", command=self.teleoperation_object,width=20, height=3).place(x=300, y=175)
		modo_6_btn = tk.Button(self.root, text="Release", command=self.release, width=20, height=3).place(x=500, y=175)
		modo_7_btn = tk.Button(self.root, text="Release position left", command=self.release_position_left, width=20, height=3).place(x=175, y=250)
		modo_8_btn = tk.Button(self.root, text="Release position right", command=self.release_position_right, width=20, height=3).place(x=425, y=250)
		#modo_7_btn = tk.Button(buttons, text="Modo 7", command=self.ejecutar_modo_7, width=10, height=5)
		ttk.Button(self.root, text="Salir", command=self.quit).pack(side=tk.BOTTOM, pady=20)

		mode = 0

		rate = rospy.Rate(10)

		while not rospy.is_shutdown():

			"""print("Indicame un modo de ejecucion")
			mode = input()

			print("mode: ", mode)"""

			# Ejecutar la ventana principal
			self.root.mainloop()

			#rospy.spin()
			rate.sleep()

		#destroy(hand)


if __name__ == '__main__':
	try: 
		manipulator_control()
	except rospy.ROSInterruptException:
		pass
