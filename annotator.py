#!/usr/bin/env python
import roslib
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import rospy
from std_msgs.msg import String
import subprocess
import signal
import os
import sys
import time
import threading
import rosbag
import yaml
import numpy as np

global bag_file, feature_file, input_topic
global pause, buff, time_buff, buff_size, counter, current, framerate, step, start_time, compressed
pause = compressed = False
counter = 0
pause_time = None
buff_size = 100
input_topic = 'camera/rgb/image_raw'

def main(argv):
	"""
	Input 		: @list argv
	Output		: -
	Description : Parses the arguments and initializes the main process loop accordingly 
	"""
	
	
	global  bag_file, feature_file, input_topic
	global  start_time, pause, buff, time_buff, buff_size, counter, current, framerate, step, compressed
	
	# Process args
	if (len(argv) > 2):
		if '-t' in argv:
			index = argv.index('-t')
			if len(argv) >= (index + 2) and not argv[index + 1].startswith('-'):
				input_topic = argv[index + 1]	
		if '-h' in argv or '-help' in argv or 'help' in argv:
			print "\nAvailable arguments"	
			print "\t: First argument, rosbag file, absolute path"
			print "\t: Second argument, result file name, relative path"
			print "\nOption:"			
			print "\t-t: Topic to be used for annotation, e.g. camera/rgb/image_raw"
			print "\nInformation on Keys:"			
			print "\tEsc: Quits"
			print "\ta: Go back 1 frame"
			print "\td: Go forward 1 frame"
			print "\tw: Writes the timestamp on the result file with id 1"
			print "\ts: Writes the timestamp on the result file with id 0"
			print "\tspace: Pause image"
			print "\t<-: Reduce playback speed"
			print "\t->: Increase playback speed"
			exit(0)		
	else:
		print"Too few arguments, type -h for more info"
		exit(0)		
	
	#Open bag and get framerate	
	bag_file = argv[1]
	bag = rosbag.Bag(bag_file)
	info_dict = yaml.load(bag._get_yaml_info())
	topics =  info_dict['topics']
	topic = topics[1]
	messages =  topic['messages']
	duration = info_dict['duration']
	topic_type = topic['type']
	
	#Checking if the topic is compressed
	if 'CompressedImage' in topic_type:
		compressed = True
		
	framerate = messages/duration
	step = framerate/5
	
	#Create results file
	feature_file = argv[2]
	if os.path.exists(feature_file):
		print 'Results file already exists'
		exit(0)
	file_obj = open(feature_file, 'a')
	
	bridge = CvBridge()
	buff = []
	time_buff = []
	
	#Loop through the rosbag
	for topic, msg, t in bag.read_messages(topics=[input_topic]):
		current = counter
		
		#Get the image
		if not compressed:
			try:
				cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
			except CvBridgeError as e:
				print(e)
		else:
			nparr = np.fromstring(msg.data, np.uint8)
			cv_image = cv2.imdecode(nparr, cv2.CV_LOAD_IMAGE_COLOR)
			
			
		if counter == 0:
			start_time = t
			
		#Append image buffer and time buffer	
		buff.append(msg)
		time_buff.append(t.to_sec() - start_time.to_sec())	
		
		#Display image
		cv2.imshow("Image", cv_image)
		keyPressed(file_obj)
		
		#If the image is paused
		while(pause):
			cv2.imshow("Image", bridge.imgmsg_to_cv2(buff[counter], "bgr8"))
			keyPressed(file_obj)
			if counter < current and not pause:
				for msg in buff[counter::]:
					cv2.imshow("Image", bridge.imgmsg_to_cv2(msg, "bgr8"))
					keyPressed(file_obj)
					if pause:
						break
					if counter < current:	
						counter += 1
					
		if len(buff) >= buff_size:
			del buff[0:10]		
		if len(time_buff) >= buff_size:
			del time_buff[0:10]		
			counter -= 10
		counter += 1
	bag.close()
	
	

def keyPressed(file_obj, key = None):
	global bag_file, prev_frame, pause, counter, time_buff, counter, current, framerate, step
	key = cv2.waitKey(int(round(1000/framerate)));
	if key == -1:
		return
	if  key & 0xFF == 27:
		cv2.destroyAllWindows()
		exit(0)	
	if  key == 1113937:
		if framerate - step > 0:
			framerate = framerate - step
	if  key == 1113939:
		framerate = framerate + step
		print framerate
	if  key & 0xFF == ord('a'):
		pause = True
		if counter == 0:
			return
		counter -= 1
	if  key & 0xFF == ord('d'):
		pause = True
		if current == counter:
			return
		counter += 1
	if  key & 0xFF == ord('s'):
		file_obj.write(str(time_buff[counter]) + "\t0\n")
	if  key & 0xFF == ord('w'):
		file_obj.write(str(time_buff[counter]) + "\t1\n")
	if  key & 0xFF == ord(' '):
		pause_time = None
		if pause is True:
			pause = False
		else:
			pause = True
	
if __name__ =='__main__':
    main(sys.argv)

