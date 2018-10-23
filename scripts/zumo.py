#!/usr/bin/env python


import bluetooth
import time
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

		self.sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )
		self.BTport=1
		self.TIMEOUT=0.1
		self.lock=Lock()
		self.message = list()
		self.p=Imu()
		self.p.header.stamp = rospy.Time.now()
		self.p.header.frame_id="map"
		self.recvd_data = ""

	def connect_to_bluetooth(self, device_name, device_mac):
		nearby_devices = bluetooth.discover_devices()
		for bdaddr in nearby_devices:
			if device_name == bluetooth.lookup_name( bdaddr ):
				device_mac = bdaddr
				break

		if device_mac is not None:
			rospy.loginfo("found target bluetooth device with address "+device_mac)
		else:
			rospy.loginfo("could not find target bluetooth device nearby")

		try:
			self.sock.connect((device_mac, self.BTport))
		except:
			rospy.loginfo("Failed to connect to"+ device_mac)

	def load_buffer(self):
		self.recvd_data += self.sock.recv(8192)

	def read_buffer(self):
		n=24
		data=""
		if len(self.recvd_data) > n*2:
			data_start = self.recvd_data.find('<')
			if data_start != 0:
				data = self.recvd_data.partition('<')
				data = data[2]
				data_end = data.find('>')
				data_msg = data[:data_end]
				self.recvd_data = data[data_end+1:]
#				rospy.loginfo("geht zuruck"+data[data_end+1:])
#				rospy.loginfo("pub "+data_msg)
				data_msg = data_msg.replace("Fakedata","")
				data_msg = data_msg.replace("\r\n","")
				self.message=data_msg.split(',')
				self.pubimu()

	def cmd_to_bluetooth(self,cmd_speed,cmd_angle):
		with self.lock:
			vmax=400 #max speed zumo
			cmd_msg="<Command,"+str(int(cmd_speed*vmax))+","+str(int(cmd_angle*vmax))+">"
			sleep(0.001)
			try :
				self.sock.send(cmd_msg)
				rospy.loginfo( "SEndMessage !"+cmd_msg)

			except :
				rospy.loginfo( "falied to convert !")

	def cb_cmdvel(self,msg):
		self.cmd_to_bluetooth(msg.linear.x,msg.angular.z)


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
	myZumo.connect_to_bluetooth("HC06", "00:21:13:00:2F:7E")
	while not rospy.is_shutdown():
#		myZumo.get_message()
		myZumo.load_buffer()
		myZumo.read_buffer()
		sleep(0.001)

	rospy.loginfo("Node terminated")
	rospy.delete_param("ZUMO_BAUDRATE")
	rospy.delete_param("ZUMO_PORT")
	rospy.loginfo("Connection lost")
