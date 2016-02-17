# rosbag_annotator

This program will help you annotate a rosbag file by producing a result file with timestamps of the annotated events.
Currently works only for Image and CompressedImage topics 

It takes as input the rosbag file path and the topic you want to view.
It playbacks the rosbag file and give you some basic controls over the display.
It allows you to pause, rewind and save the current timestamp to the result file.
Four buttons exist for annotating events, each with its own id. 
The produced file is a simple text file with 2 collumns with the same name as the rosbag file ending in _RES.
The 1st column contains the timestamp of each annotated event and the 2nd the id of the event.

Program controls:			

| Key | Description          |
| ------------- | -----------|
| a     | Go forward 1 frame |
| d     | Go back 1 frame    |
| s     | Saves timestamp with id 0 |
| w     | Saves timestamp with id 1 |
| q     | Saves timestamp with id 2 |
| e     | Saves timestamp with id 3 |
| z     | Saves timestamp with id 4 |
| space | Pauses program     |
| left arrow     | Reduce playback speed |
| right arrow     | Increase playback speed |
| Esc     | Quits program      |

For options and help execute with -h
