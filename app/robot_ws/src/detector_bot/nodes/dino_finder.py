#!/usr/bin/env python3
import rospy
rospy.init_node('dino_detect', log_level=rospy.DEBUG)
import sys
rospy.loginfo(sys.path)

import torch
import torchvision
import torch.nn.functional as F
import time
import datetime
import numpy as np
import cv2
import torchvision.transforms as transforms
import PIL.Image
import boto3
import os
import logging
logging.basicConfig(level=logging.DEBUG)
from jetbot import Camera

steering_model_path = rospy.get_param("ml_model_directory")+rospy.get_param("steering_model")
dino_detect_model_path = rospy.get_param("ml_model_directory")+rospy.get_param("dino_detect_model")

try: 
    model_roadfollow = torchvision.models.resnet18(pretrained=False)
    model_roadfollow.fc = torch.nn.Linear(512, 2)
    model_roadfollow.load_state_dict(torch.load(steering_model_path))
    
    model_dinodet = torchvision.models.resnet18(pretrained=False)
    model_dinodet.fc = torch.nn.Linear(512, 6)
    model_dinodet.load_state_dict(torch.load(dino_detect_model_path))
    
    device = torch.device('cuda')
    
    model = model_roadfollow.to(device)
    model = model_roadfollow.eval().half()
    
    model_dinodet = model_dinodet.to(device)
    model_dinodet = model_dinodet.eval()
    
    camera = Camera.instance(width=int(rospy.get_param("image_width")), height=int(rospy.get_param("image_height")))

    from jetbot import Robot
    robot = Robot()
    speed_gain_slider = float(rospy.get_param("speed"))
    steering_gain_slider = float(rospy.get_param("steering_gain"))
    steering_dgain_slider = float(rospy.get_param("steering_d_gain"))
    steering_bias_slider = float(rospy.get_param("steering_bias"))
    angle = 0.0
    angle_last = 0.0
    prev_class = -1
except Exception as e:
    rospy.loginfo("Error occured:", str(e))
    
dino_names = rospy.get_param("dinosaurs")

mean = 255.0 * np.array([0.485, 0.456, 0.406])
stdev = 255.0 * np.array([0.229, 0.224, 0.225])

mean_roadfollow = torch.Tensor([0.485, 0.456, 0.406]).cuda().half()
std_roadfollow = torch.Tensor([0.229, 0.224, 0.225]).cuda().half()

normalize = torchvision.transforms.Normalize(mean, stdev)

def push_to_s3(img):
    try:
        bucket_name = "dinobot-roscon-unknown"
        timestamp = int(time.time())
        now = datetime.datetime.now()
        key = "unknown_dino/{}_{}_{}_{}_{}.jpg".format(now.month, now.day, now.hour, now.minute, timestamp)

        #s3 = boto3.client('s3')

        #encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        #_, jpg_data = cv2.imencode('.jpg', img, encode_param)
        #response = s3.put_object(ACL='public-read',
        #                         Body=jpg_data.tostring(),
        #                         Bucket=bucket_name,
        #                         Key=key)
        msg = key
        print("pub to s3:"+key)
    except Exception as e:
        msg = "Pushing to S3 failed: " + str(e)
    return msg

def publish_to_iot(msg):
    print("publish to iot: "+msg)
#    try:
#        client = boto3.client('iot-data')
#        response = client.publish(topic ='dino-detect' , qos =0, payload=msg)
#    except Exception as e:
#        msg = "Publish to IoT failed: " + str(e)


def preprocess(camera_value):
    global device, normalize
    x = camera_value
    x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB)
    x = x.transpose((2, 0, 1))
    x = torch.from_numpy(x).float()
    x = normalize(x)
    x = x.to(device)
    x = x[None, ...]
    return x

def preprocess_roadfollow(image):
    image = PIL.Image.fromarray(image)
    image = transforms.functional.to_tensor(image).to(device).half()
    image.sub_(mean_roadfollow[:, None, None]).div_(std_roadfollow[:, None, None])
    return image[None, ...]

def find_dino(change):
    x = change['new'] 
    x = preprocess(x)
    y = model_dinodet(x)
    y_dino = F.softmax(y, dim=1)
    topk = y_dino.cpu().topk(1)
    
    return (e.data.numpy().squeeze().tolist() for e in topk)


def move_bot(image, robot_stop):
    global angle, angle_last    
    if robot_stop:
        robot.stop()
        robot.left_motor.value=0
        robot.left_motor.value=0
        time.sleep(2)
        robot_stop = False
    else:
        xy = model_roadfollow(preprocess_roadfollow(image)).detach().float().cpu().numpy().flatten()
        x = xy[0]
        y = (0.5 - xy[1]) / 2.0
        speed_slider = speed_gain_slider
        angle = np.arctan2(x, y)
        pid = angle * steering_gain_slider + (angle - angle_last) * steering_dgain_slider
        angle_last = angle
        steering_slider = pid + steering_bias_slider
        robot.left_motor.value = max(min(speed_slider + steering_slider, 1.0), 0.0)
        robot.right_motor.value = max(min(speed_slider - steering_slider, 1.0), 0.0)
    
while True:
    try: 
        img = camera.value
        robot_stop = False
        probs, classes = find_dino({'new': img}) 
        msg = "Start..."
        s3url = ""
        if probs > 0.6 and prev_class != classes:
            prev_class = classes
            robot_stop = True
            if classes == 5:
                s3url = push_to_s3(img)
                msg = '{"dinosaur":"unknown"' + ',"confidence":"' + str(probs) +'","url":"'+ s3url +'"}'
            else:
                msg = '{"dinosaur":"' + dino_names[classes] + '"' + ',"confidence":"' + str(probs) +'"}'
            print(msg)
            publish_to_iot(msg)
        move_bot(img, robot_stop)
    except Exception as e:
        rospy.loginfo("Error occured:", str(e))