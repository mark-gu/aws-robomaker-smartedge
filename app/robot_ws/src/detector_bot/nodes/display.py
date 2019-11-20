#!/usr/bin/env python2
## From dusty_nv
import rospy
import time

import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

import subprocess

from std_msgs.msg import String

def get_ip_address(interface):
    if get_network_interface_state(interface) == 'down':
        return None
    cmd = "ifconfig %s | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'" % interface
    return subprocess.check_output(cmd, shell=True).decode('ascii')[:-1]


def get_network_interface_state(interface):
    return subprocess.check_output('cat /sys/class/net/%s/operstate' % interface, shell=True).decode('ascii')[:-1]

user_text=None

def on_user_text(msg):
	global user_text	
	user_text=msg.data

	rospy.loginfo(rospy.get_caller_id() + ' user_text=%s', msg.data)

# initialization
if __name__ == '__main__':

	# 128x32 display with hardware I2C:
	disp = Adafruit_SSD1306.SSD1306_128_64( rst=None, i2c_bus=1, gpio=1) # wes modification

	# Initialize library.
	disp.begin()

	# Clear display.
	disp.clear()
	disp.display()

	# Create blank image for drawing.
	# Make sure to create image with mode '1' for 1-bit color.
	width = disp.width
	height = disp.height
	image = Image.new('1', (width, height))

	# Get drawing object to draw on image.
	draw = ImageDraw.Draw(image)

	# Draw a black filled box to clear the image.
	draw.rectangle((0,0,width,height), outline=0, fill=0)

	# Draw some shapes.
	# First define some constants to allow easy resizing of shapes.
	padding = -2
	top = padding
	bottom = height-padding
	# Move left to right keeping track of the current x position for drawing shapes.
	x = 0

	# Load default font.
	font1 = ImageFont.truetype("FreeSans.ttf", 9)
	font2 = ImageFont.truetype("FreeSans.ttf", 11)
    
	# setup ros node
	rospy.init_node('display')
	rospy.Subscriber('~user_text', String, on_user_text)

	# start running
	while not rospy.core.is_shutdown():
            
		# Draw a black filled box to clear the image.
		draw.rectangle((0,0,width,height), outline=0, fill=0)

		# Shell scripts for system monitoring from here : https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
		#cmd = "top -bn1 | grep load | awk '{printf \"CPU Load: %.2f\", $(NF-2)}'"
		#CPU = subprocess.check_output(cmd, shell = True )
		cmd = "top -bn1 | grep load | awk '{printf \"CPU Load:  %.2f\", $(NF-2)}'"
		CPU = subprocess.check_output(cmd, shell = True )
		cmd = "free -m | awk 'NR==2{printf \"%.2f%%\", $3*100/$2}'"
		Mem_percent = subprocess.check_output(cmd, shell = True )
		cmd = "free -m | awk 'NR==2{printf \"%s/%sMB\", $3,$2}'"
		MemUsage = subprocess.check_output(cmd, shell = True )
		cmd = "df -h | awk '$NF==\"/\"{printf \"%s\", $5}'"
		Disk_percent = subprocess.check_output(cmd, shell = True )
		cmd = "df -h | awk '$NF==\"/\"{printf \"%d/%d GB\", $3,$2}'"
		DiskUsage = subprocess.check_output(cmd, shell = True )
	    
		# Write two lines of text.
		if not user_text is None:
			draw.text((x+32, top+16), user_text,  font=font2, fill=255)
		else:
			draw.text((x+32, top+16),    str(CPU.decode('utf-8')),  font=font1, fill=255)
			draw.text((x+32, top+24),    "Mem:  " + str(Mem_percent.decode('utf-8')),  font=font1, fill=255)
			draw.text((x+32, top+34),    "Disk:  " + str(Disk_percent.decode('utf-8')),  font=font1, fill=255)
			draw.text((x+32, top+44),    "wlan0: ",  font=font1, fill=255)
			draw.text((x+31, top+54),    str(get_ip_address('wlan0')), font=font1, fill=255)
	    
		# Display image.
		disp.image(image)
		disp.display()
		
		# Update ROS
		rospy.rostime.wallsleep(1.0)
