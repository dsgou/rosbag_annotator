#!/usr/bin/env python
import roslib
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import rospy
from std_msgs.msg import String
import signal
import os
import sys
import time
import threading
import rosbag
import yaml
import numpy as np
import argparse
import textwrap

global pause
global framerate
global step
global counter
global current
global mouse_pressed
global mouse_loc
global prev_mouse_loc
global start_rect

pause = False
mouse_pressed = False
mouse_loc = None
prev_mouse_loc = None
start_rect = 2*[None]

def parse_arguments():
	parser = argparse.ArgumentParser(
		prog='PROG',
		formatter_class=argparse.RawDescriptionHelpFormatter,
		description=textwrap.dedent('''\
		This script annotates a rosbag file and creates a result file,
		the following keys can be used for annotation and control
		\tEsc: Quits
		\ta: Go back 1 frame
		\td: Go forward 1 frame
		\tz: Writes the timestamp on the result file with id 4
		\te: Writes the timestamp on the result file with id 3
		\tq: Writes the timestamp on the result file with id 2
		\tw: Writes the timestamp on the result file with id 1
		\ts: Writes the timestamp on the result file with id 0
		\tspace: Pause image
		\t<-: Reduce playback speed
		\t->: Increase playback speed
		'''))
	parser.add_argument('-i', '--input-file',    required=True,  nargs='?', help="rosbag file, absolute path")
	parser.add_argument('-vt', '--visual-topic', required=True,  nargs='?', help="topic to be used for visual annotation, e.g. /camera/rgb/image_raw")
	parser.add_argument('-c', '--csv-file',  nargs='?', help="csv file with bounded boxes of the bag played")
	parser.add_argument('-o', '--output-file',  nargs='?', help="output annotation result file")
	parser.add_argument('-a', '--append', default=False, help="append result file instead of creating new", action='store_true')
	return parser.parse_args()


def mouse_cb(event, x , y, flags, param):
	global mouse_pressed
	global mouse_loc
	global prev_mouse_loc
	global start_rect
	
	if event == cv2.EVENT_LBUTTONDOWN:
		if mouse_pressed:
			prev_mouse_loc = mouse_loc
			start_rect[0] = prev_mouse_loc
			start_rect[1] = (x, y)
		mouse_loc = (x, y)
		mouse_pressed = not mouse_pressed
		
	if flags == (cv2.EVENT_FLAG_CTRLKEY + cv2.EVENT_FLAG_LBUTTON):	
		print x,y
	
def setCounter(x, counter):
	counter = x	
	print current
	
def play_bag_file(bag_file, csv_file):
	global pause
	global framerate
	global step
	global counter
	global current
	global mouse_pressed
	global mouse_loc
	global prev_mouse_loc
	global start_rect
	
	bag = rosbag.Bag(bag_file)
	info_dict = yaml.load(bag._get_yaml_info())
	topics =  info_dict['topics']
	topic = topics[1]
	messages =  topic['messages']
	duration = info_dict['duration']
	topic_type = topic['type']
		
	#Messages for test
	print "Script parameters: ","\n\t- Bag file: ", bag_file, "\n\t- Topic: ", input_topic, 
	print "\nRosbag topics found: "
	for top in topics:
		print "\t- ", top["topic"], "\n\t\t-Type: ", topic["type"],"\n\t\t-Fps: ", topic["frequency"]
		
	#Checking if the topic is compressed
	if 'CompressedImage' in topic_type:
		compressed = True
	else:
		compressed = False
		
	#Get framerate	
	framerate = messages/duration
	step = framerate/5
	
	bridge = CvBridge()
	image_buff = []
	time_buff = []
	box_buff = []
	counter = 0
	buff_size = messages
	file_obj = open(feature_file, 'a')
	
	cv_image = None
	cv2.namedWindow("Image");
	cv2.setMouseCallback("Image", mouse_cb)
	cv2.createTrackbar('Progress', 'Image', 0, messages, setCounter)
	
	if csv_file is not None and os.path.exists(csv_file):
		csv = open(csv_file, 'r')
		index = 0
		line = csv.readline()
		for field in line.split('\t'):
			if 'Rect_x' in field:
				break
			index += 1
			
	#Loop through the rosbag
	for topic, msg, t in bag.read_messages(topics=[input_topic]):
		if counter == 0:
			start_time = t
			
		#Get the image
		if not compressed:
			try:
				cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
			except CvBridgeError as e:
				print e
		else:
			nparr = np.fromstring(msg.data, np.uint8)
			cv_image = cv2.imdecode(nparr, cv2.CV_LOAD_IMAGE_COLOR)
			
		try:
			line = csv.readline()
			(x, y, width, height) = map(int, line.split('\t')[index:index + 4])
			box_buff.append((x, y, width, height))		
		except Exception as e:
			pass
				
		image_buff.append(cv_image)
		time_buff.append(t.to_sec() - start_time.to_sec())
		counter += 1
	
	
			
	counter = 0
	
	#Loop through the image buffer
	for current in range(len(image_buff)):
		current = counter
		cv_image = image_buff[current].copy()
		
		try:
			(x, y, width, height) = box_buff[current]
			cv2.rectangle(cv_image, (x, y), ((x + width), (y + height)), (255, 0, 0), 1)	
			if start_rect[0] != None and start_rect[1] != None:
				cv2.rectangle(cv_image, start_rect[0], start_rect[1], (255, 0, 0), 1)
		except Exception as e:
			pass
			
		#Display image
		cv2.imshow("Image", cv_image)
		keyPressed(time_buff, file_obj)

		#If the image is paused
		while(pause):
			cv_image_pause = image_buff[counter].copy()
			if start_rect[0] != None and start_rect[1] != None:
				box_buff[counter] = (start_rect[0][0], start_rect[0][1], start_rect[1][0] - start_rect[0][0], start_rect[1][1]- start_rect[0][1])
			
			try:
				(x, y, width, height) = box_buff[counter]
				cv2.rectangle(cv_image_pause, (x, y), ((x + width), (y + height)), (255, 0, 0), 1)	
			except Exception as e:
				pass
				
			cv2.imshow("Image", cv_image_pause)
			keyPressed(time_buff, file_obj)
			if counter < current and not pause:
				for cv_image_pause in image_buff[counter::]:
					try:					
						(x, y, width, height) = box_buff[counter]
						cv2.rectangle(cv_image_pause, (x, y), ((x + width), (y + height)), (255, 0, 0), 1)	
					except Exception as e:
						pass
					cv2.imshow("Image", cv_image_pause)
					keyPressed(time_buff, file_obj)
					if pause:
						break
					if counter < current:	
						counter += 1
					start_rect = 2*[None]
		counter += 1
		start_rect = 2*[None]
	
	if csv_file is not None and os.path.exists(csv_file):	
		input_file  = open(csv_file, 'r')
		output_file = open(csv_file.split(".")[0] + "_out.csv", 'w')
		for line, i in zip(input_file, range(len(box_buff))):
			print map(str, box_buff[i])
			line.split('\t')[index:index + 4] = map(str, box_buff[i])
			output_file.write(line)
			
	bag.close()
	
def keyPressed(time_buff, file_obj, key = None):
	global pause
	global framerate
	global step
	global counter
	global current
	
	key = cv2.waitKey(int(round(1000/framerate)));
	if key == -1:
		return
	if  key & 0xFF == 27:
		cv2.destroyAllWindows()
		exit(0)	
	if  key == 1113937 or key == 65361:
		if framerate - step > 0:
			framerate = framerate - step
	if  key == 1113939 or key == 65363:
		framerate = framerate + step
	if  key & 0xFF == ord('a'):
		pause = True
		if counter == 0:
			return
		counter -= 1
		print counter
	if  key & 0xFF == ord('d'):
		pause = True
		if current == counter:
			return
		counter += 1
	if  key & 0xFF == ord('s'):
		file_obj.write(str(time_buff[counter]) + "\t0\n")
	if  key & 0xFF == ord('w'):
		file_obj.write(str(time_buff[counter]) + "\t1\n")
	if  key & 0xFF == ord('q'):
		file_obj.write(str(time_buff[counter]) + "\t2\n")
	if  key & 0xFF == ord('e'):
		file_obj.write(str(time_buff[counter]) + "\t3\n")
	if  key & 0xFF == ord('z'):
		file_obj.write(str(time_buff[counter]) + "\t4\n")
	if  key & 0xFF == ord(' '):
		pause_time = None
		if pause is True:
			pause = False
		else:
			pause = True
			

if __name__ =='__main__':
	args = parse_arguments()
	bag_file = args.input_file
	csv_file = args.csv_file
	output_file = args.output_file
	input_topic = args.visual_topic
	append = args.append
	
	#Create results file
	if(output_file is None):
		feature_file = bag_file.split(".")[0].split("/")[-1] + "_RES"
	else:
		feature_file = output_file
	
	if os.path.exists(feature_file) and not append:
		os.remove(feature_file)
		
	print feature_file	
	
	#Open bag and get framerate	
	play_bag_file(bag_file, csv_file)

