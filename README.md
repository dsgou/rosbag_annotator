# rosbag_annotator

This repository is a tool that helps you annotate a rosbag file topic by producing a result file with timestamps.
Currently works only for image topics 

It takes as input the rosbag file path, the results file path and the topic you want to view.
It playbacks the rosbag file and allows you to pause, rewind and save the current timestamp to the result file.
So the produced file is simply a text with timestamps of the annotated event.

Information on Keys:			

| Key | Description          |
| ------------- | -----------|
| a     | Go forward 1 frame |
| d     | Go back 1 frame    |
| s     | Saves timestamp    |
| space | Pauses program     |
| q     | Quits program      |

For options and help execute with -h
