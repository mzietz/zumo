#!/usr/bin/env python

import cv2
import sys
import rospy
import numpy as np
import random as rng


class Vision():
	def __init__(self):
		self.faceCascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
		self.video_capture = cv2.VideoCapture(0)
		self.orb = cv2.ORB_create()

	def face_detector(self):

		# Capture frame-by-frame
		ret, frame = self.video_capture.read()

		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

		faces = self.faceCascade.detectMultiScale(
			gray,
			scaleFactor=1.1,
			minNeighbors=5,
			minSize=(30, 30)
		)

		# Draw a rectangle around the faces
		for (x, y, w, h) in faces:
			cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
			print("Gesicht Koordinaten: X = "+str(x+w/2)+" Y = "+str(y+h/2))
		# Display the resulting frame
		cv2.imshow('Video', frame)

	def line_detector(self):

		# Capture frame-by-frame
		ret, frame = self.video_capture.read()
		gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
		kernel_size = 5
		blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size),0)
		low_threshold = 50
		high_threshold = 150
		edges = cv2.Canny(blur_gray, low_threshold, high_threshold)
		
		rho = 1  # distance resolution in pixels of the Hough grid
		theta = np.pi / 180  # angular resolution in radians of the Hough grid
		threshold = 15  # minimum number of votes (intersections in Hough grid cell)
		min_line_length = 50  # minimum number of pixels making up a line
		max_line_gap = 20  # maximum gap in pixels between connectable line segments
		line_image = np.copy(frame) * 0  # creating a blank to draw lines on

		# Run Hough on edge detected image
		# Output "lines" is an array containing endpoints of detected line segments
		lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
							min_line_length, max_line_gap)

		for line in lines:
			for x1,y1,x2,y2 in line:
				cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),5)
		lines_edges = cv2.addWeighted(frame, 0.8, line_image, 1, 0)
		cv2.imshow('Video', lines_edges)

	def orb_detector(self):
		ret, frame = self.video_capture.read()
		ret2, frame2 = self.video_capture.read()
		resized = cv2.resize(frame, None, fx = 0.5, fy = 0.5, interpolation = cv2.INTER_AREA)
		# find the keypoints with ORB
		kp = self.orb.detect(frame,None)
		# compute the descriptors with ORB
		kp, des = self.orb.compute(frame, kp)
		# draw only keypoints location,not size and orientation
		frame2 = cv2.drawKeypoints(frame, kp, None, color=(0,255,0), flags=0)
		cv2.imshow("Video", frame)
		cv2.imshow("Video2", resized)

	def canny_alg(self):
		ret, frame = self.video_capture.read()
		edges = cv2.Canny(frame,100,200)
		laplacian = cv2.Laplacian(frame,cv2.CV_64F)
		rho = 1  # distance resolution in pixels of the Hough grid
		theta = np.pi / 180  # angular resolution in radians of the Hough grid
		threshold = 20  # minimum number of votes (intersections in Hough grid cell)
		min_line_length = 100  # minimum number of pixels making up a line
		max_line_gap = 20  # maximum gap in pixels between connectable line segments
		line_image = np.copy(frame) * 0  # creating a blank to draw lines on

		lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)
		for x in range(0, len(lines)):
			for x1,y1,x2,y2 in lines[x]:
				cv2.line(frame,(x1,y1),(x2,y2),(255,0,0),2)
		cv2.imshow("Video1", frame)
		cv2.imshow("Video2", edges)

	def segmentation(self):
		ret, frame = self.video_capture.read()
		# find the keypoints with ORB
		frame = cv2.resize(frame, None, fx = 0.5, fy = 0.5, interpolation = cv2.INTER_AREA)
		kernel = np.array([[1, 1, 1], [1, -8, 1], [1, 1, 1]], dtype=np.float32)
		imgLaplacian = cv2.filter2D(frame, cv2.CV_32F, kernel)
		sharp = np.float32(frame)
		imgResult = sharp - imgLaplacian
		# convert back to 8bits gray scale
		imgResult = np.clip(imgResult, 0, 255)
		imgResult = imgResult.astype('uint8')
		imgLaplacian = np.clip(imgLaplacian, 0, 255)
		imgLaplacian = np.uint8(imgLaplacian)
		#cv.imshow('Laplace Filtered Image', imgLaplacian)

		bw = cv2.cvtColor(imgResult, cv2.COLOR_BGR2GRAY)
		_, bw = cv2.threshold(bw, 100, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
		dist = cv2.distanceTransform(bw, cv2.DIST_L2, 3)
		# Normalize the distance image for range = {0.0, 1.0}
		# so we can visualize and threshold it
		cv2.normalize(dist, dist, 0, 1.0, cv2.NORM_MINMAX)
		_, dist = cv2.threshold(dist, 0.4, 1.0, cv2.THRESH_BINARY)
		# Dilate a bit the dist image
		kernel1 = np.ones((3,3), dtype=np.uint8)
		dist = cv2.dilate(dist, kernel1)

		dist_8u = dist.astype('uint8')
		# Find total markers
		_, contours, _ = cv2.findContours(dist_8u, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		# Create the marker image for the watershed algorithm
		markers = np.zeros(dist.shape, dtype=np.int32)
		# Draw the foreground markers
		for i in range(len(contours)):
			cv2.drawContours(markers, contours, i, (i+1), -1)
		# Draw the background marker
		cv2.circle(markers, (5,5), 3, (255,255,255), -1)

		cv2.watershed(imgResult, markers)
		#mark = np.zeros(markers.shape, dtype=np.uint8)
		mark = markers.astype('uint8')
		mark = cv2.bitwise_not(mark)
		# uncomment this if you want to see how the mark
		# image looks like at that point
		#cv2.imshow('Markers_v2', mark)
		# Generate random colors
		colors = []
		for contour in contours:
			colors.append((rng.randint(0,256), rng.randint(0,256), rng.randint(0,256)))
		# Create the result image
		dst = np.zeros((markers.shape[0], markers.shape[1], 3), dtype=np.uint8)
		# Fill labeled objects with random colors
		for i in range(markers.shape[0]):
			for j in range(markers.shape[1]):
				index = markers[i,j]
				if index > 0 and index <= len(contours):
					dst[i,j,:] = colors[index-1]
		# Visualize the final image
		cv2.imshow('bw', bw)
		cv2.imshow('dist', dist)
		cv2.imshow('dst', dst)

	def contour_detector(self):
		ret, frame = self.video_capture.read()
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		gray_blur = cv2.GaussianBlur(gray, (15, 15), 0)
		thresh = cv2.adaptiveThreshold(gray_blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 1)
		kernel = np.ones((3, 3), np.uint8)
		closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=4)
		cont_img = closing.copy()
		image, contours , hierarchy= cv2.findContours(cont_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#		cv2.imshow("Morphological Closing", closing)
		for cnt in contours:
			area = cv2.contourArea(cnt)
			if area < 2000 or area > 4000:
				continue
			if len(cnt) < 5:
				continue
			[x1,y1,x2,y2] = cv2.fitLine(cnt, cv2.DIST_L2,0,0.01, 0.01)
			cv2.line(frame, (x1,y1), (x2,y2), (255,0,0), 5)
			
		cv2.imshow("Adaptive Thresholding", frame)
#		cv2.imshow('Contours', frame)


if __name__ == '__main__':
	myVision= Vision()
	while True:
		myVision.canny_alg()
#		myVision.line_detector()
#		myVision.segmentation()
#		myVision.orb_detector()
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
	myVision.video_capture.release()
	cv2.destroyAllWindows()
