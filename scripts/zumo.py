#!/usr/bin/env python


import serial
from math import sqrt
import rospy
from time import sleep
from threading import Lock
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class Zumo:
	"""docstring for Zumo"""
	def __init__(self):
		self.sub_cmd=rospy.Subscriber("/cmd_vel",Twist,self.cb_cmdvel)
		self.pub_imu=rospy.Publisher("/imu", Imu, queue_size=10)
		try:
			self.PORT=rospy.get_param('ZUMO_PORT')
		except:
			rospy.set_param('ZUMO_PORT',"dev/ttyACM0")
			self.PORT=rospy.get_param('ZUMO_PORT')
		try:
			self.BAUDRATE=rospy.get_param('ZUMO_BAUDRATE') 
		except:
			rospy.set_param('ZUMO_BAUDRATE',"9600")
			self.BAUDRATE=rospy.get_param('ZUMO_BAUDRATE')
		self.TIMEOUT=0.1
		self.lock=Lock()
		self.message = list()
		self.p=Imu()
		self.p.header.stamp = rospy.Time.now()
		self.p.header.frame_id="map"
		try:
			self.ser = serial.Serial( self.PORT, self.BAUDRATE,timeout=self.TIMEOUT)
			sleep(1)
			rospy.loginfo("Connection established at "+str(self.PORT))

		except:
			rospy.loginfo("Try to connect")
			
	def __delete__(self):
		self.ser.close()
		print "Connection lost"
		
	def get_message(self):
		with self.lock: 
			try:
				self.ser.flush()
				sleep(0.001)
				line = self.ser.readline() # reception trame accelero/magneto/gyro
				if len(line)>0:
					if line.startswith('!AN'):
						line = line.replace("!AN:", "")
						line = line.replace("\r\n", "")
						self.message=line.split(',')
						#rospy.loginfo( "Got message : "+str(self.message))
						self.pubimu()
			except:
				rospy.loginfo("Failed getting message !")

	def cmd_to_serial(self,cmd_speed,cmd_angle):
		with self.lock:
			vmax=400 #max speed zumo
			#cmd_msg="~X;"+str(int (cmd_speed*100))+";"+str(int (cmd_angle*100))+";#"
			cmd_msg="<Command,"+str(int(cmd_speed*vmax))+","+str(int(cmd_angle*vmax))+">"
			self.ser.flush()
			sleep(0.001)
			try :
				self.ser.write(cmd_msg)
				rospy.loginfo(cmd_msg)
			except :
				rospy.loginfo( "falied to convert !")

	def cb_cmdvel(self,msg):
		self.cmd_to_serial(msg.linear.x,msg.angular.z)
		#rospy.loginfo("Got message !")


	def pubimu(self):
		self.p.linear_acceleration.x= (4*9.81*float(self.message[1])/2**16)
		self.p.linear_acceleration.y=(4*9.81*float(self.message[2])/2**16)
		self.p.linear_acceleration.z=(4*9.81*float(self.message[3])/2**16)
		self.p.orientation.x= float(self.message[4])
		self.p.orientation.y=float(self.message[5])
		self.p.orientation.z=float(self.message[6])
		self.pub_imu.publish(self.p)

if __name__=="__main__":

	print "Starting"
	rospy.init_node("zumo")
	myZumo=Zumo()

	while not rospy.is_shutdown():
		myZumo.get_message()
		sleep(0.001)

	rospy.loginfo("Node terminated")
	rospy.delete_param("ZUMO_BAUDRATE")
	rospy.delete_param("ZUMO_PORT")
	myZumo.ser.close()
	rospy.loginfo("Connection lost")
