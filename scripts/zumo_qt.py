#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
ZetCode PyQt5 tutorial 

This example shows a tooltip on 
a window and a button.

Author: Jan Bodnar
Website: zetcode.com 
Last edited: August 2017
"""

import sys
import rospy
from PyQt5.QtWidgets import (QWidget, QToolTip, 
    QPushButton, QApplication)
from PyQt5.QtGui import QFont 
from PyQt5.QtCore import QProcess
from zumo_pi import Zumo


class Example(QWidget):
    
    def __init__(self):
        super().__init__()
        
        self.initUI()
        
        
    def initUI(self):
        
        QToolTip.setFont(QFont('SansSerif', 10))
        
        self.setToolTip('This is a <b>QWidget</b> widget')
        
        btn = QPushButton('Button', self)
        btn.setToolTip('This is a <b>QPushButton</b> widget')
        btn.resize(btn.sizeHint())
        btn.move(50, 50) 
        btn.clicked[bool].connect(self.launchROS)

        blueb = QPushButton('Blue', self)
        blueb.setCheckable(True)
        blueb.move(10, 110)

        blueb.clicked[bool].connect(self.launchROS)

        self.setGeometry(300, 300, 300, 200)
        self.setWindowTitle('Tooltips')    
        self.show()
        


    def launchROS(self, pressed):
        
        source = self.sender()
       
        if pressed:
        	print ("Launching...")
        	ROS_PROGRAM = QProcess(self)

#        	program = "gnome-terminal --geometry=50x10-0-10 -x bash -c \"roslaunch zumo zumo_pi.launch\" "
        	ROS_PROGRAM.start("gnome-terminal --geometry=50x10-0-10 -x sh bash -c \"roslaunch zumo zumo_pi.launch\" ")
       		print("leider nein")
if __name__=="__main__":

	print "Starting"
	app = QApplication(sys.argv)
	print ("Launching...")
	ex = Example()
	sys.exit(app.exec_())
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


