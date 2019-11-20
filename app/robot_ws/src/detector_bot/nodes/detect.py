#!/usr/bin/env python3
import torch
import torchvision
import torch.nn.functional as F
import time
import datetime
import numpy as np
import cv2
import torchvision.transforms as transforms
import base64
import json
import random
import logging
# from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import PIL.Image

# IOT_HOST = "a8hgrxh4eewrt-ats.iot.us-east-1.amazonaws.com"
DEBUG = True
device = None
config = {
    "i2c_bus": 0,
    "speed_gain_slider": 0.27,
    "steering_gain_slider": 0.05,
    "steering_dgain_slider": 0.00,
    "steering_bias_slider": -0.01,
    "prev_class": -1,
    "mean_values": [0.485, 0.456, 0.406],
    "std_values": [0.229, 0.224, 0.225],
    "np_value": 255.0,
    "mascot_names": ['0', '1', '3', '4', '5', '6', '7', '8', '9'],
    "topic": "object-detect",
    "road_following_model": "/tmp/trained_models/best_steering_model_xy.pth",
    "obj_detect_model": "/tmp/trained_models/best_mascotdet_model_xy.pth",
    "image_size": [224, 224]
}
settings = {"mean_roadfollow": None, "std_roadfollow": None}

mean = config['np_value'] * np.array(config['mean_values'])
stdev = config['np_value'] * np.array(config['std_values'])
normalize = torchvision.transforms.Normalize(mean, stdev)
mean_roadfollow = torch.Tensor([0.485, 0.456, 0.406]).cuda().half()
std_roadfollow = torch.Tensor([0.229, 0.224, 0.225]).cuda().half()
model_objdet = None
model_roadfollow = None

angle = 0.0
angle_last = 0.0
iotClient = None

bridge = CvBridge()


def preprocess(camera_value):
    global device, settings, normalize
    x = camera_value
    #print(x)
    x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB)
    x = x.transpose((2, 0, 1))
    x = torch.from_numpy(x).float()
    x = normalize(x)
    x = x.to(device)
    x = x[None, ...]
    return x


def preprocess_roadfollow(image):
    global device, settings
    image = PIL.Image.fromarray(image)
    image = transforms.functional.to_tensor(image).to(device).half()
    image.sub_(mean_roadfollow[:, None, None]).div_(std_roadfollow[:, None, None])
    return image[None, ...]


def find_obj(change):
    global model_objdet
    x = change['new']
    x = preprocess(x)
    y = model_objdet(x)
    y_obj = F.softmax(y, dim=1)
    topk = y_obj.cpu().topk(1)
    return (e.data.numpy().squeeze().tolist() for e in topk)


def move_bot(image, robot_stop=False):
    global config, angle, angle_last, model_roadfollow
    pub = rospy.Publisher('move/cmd_raw', String)

    if robot_stop:
        pub.publish(json.dumps({'left': 0, 'right': 0}))
        time.sleep(2)
        return

    processed_image = preprocess_roadfollow(image)
    xy = model_roadfollow(preprocess).detach().float().cpu().numpy().flatten()
    x = xy[0]
    y = (0.5 - xy[1]) / 2.0

    angle = np.arctan2(x, y)

    pid = angle * config['steering_gain_slider'] + (angle -
                                                    angle_last) * config['steering_dgain_slider']

    angle_last = angle
    steering_slider = pid + config['steering_bias_slider']
    move_data = {
        'left': max(min(config['speed_gain_slider'] + steering_slider, 1.0), 0.0),
        'right': max(min(config['speed_gain_slider'] - steering_slider, 1.0), 0.0)
    }

    pub.publish(json.dumps(move_data))

def process_objects(img):
    global config, settings, device, iotClient
    probs, classes = find_obj({'new': img})
    prev_class = None
    if probs > 0.6 and prev_class != classes:
        prev_class = classes
        robot_stop = True
        if classes == 5:
            rospy.loginfo("Found unknown object...")
            message = {
                "object": "unknown",
                "confidence": str(probs),
                "image": base64.b64encode(PIL.Image.fromarray(img))
            }
        else:
            rospy.loginfo("Found %s...", config['mascot_names'][classes])
            message = {
                "object": config['mascot_names'][classes],
                "confidence": str(probs),
                "image": base64.b64encode(Image.fromarray(img))
            }
    #     iotClient.publish(config['topic'], json.dumps(message), 1)


def callback(data):
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    robot_stop = False
    process_objects()
    move_bot(img, robot_stop)


def detect():
    global config, settings, device, iotClient, model_objdet, model_roadfollow

    if (DEBUG):
        rospy.init_node('obj_detect', log_level=rospy.DEBUG)
    else:
        rospy.init_node('obj_detect', log_level=rospy.ERROR)

    # Initialize the JetBot Robot.
    rospy.loginfo("Starting cuda...")
    device = torch.device('cuda')
    rospy.loginfo("Initializing robot on I2C Bus %i...", config['i2c_bus'])

    rospy.loginfo("Initializing ML models...")
    rospy.loginfo("Road following model...")
    model_roadfollow = torchvision.models.resnet18(pretrained=False)
    model_roadfollow.fc = torch.nn.Linear(512, 2)
    model_roadfollow.load_state_dict(torch.load(config['road_following_model']))

    model = model_roadfollow.to(device)
    model = model_roadfollow.eval().half()

    # rospy.loginfo("Object detection model...")
    # model_objdet = torchvision.models.resnet18(pretrained=False)
    # model_objdet.fc = torch.nn.Linear(512, 6)
    # model_objdet.load_state_dict(torch.load(config['obj_detect_model']))
    #
    # model_objdet = model_objdet.to(device)
    # model_objdet = model_objdet.eval()

    prev_class = config['prev_class']

    # Initialize the AWS IoT Connection based on AWS Greengrass config.
    rospy.loginfo("Initializing AWS IoT...")
    #iotClient = None
    #iotClient = AWSIoTMQTTClient('jetbot'+str(random.randint(1,101))
    #iotClient.configureEndpoint(IOT_HOST, 8843)
    #iotClient.configureCredentials("../../share/jetbot/certs/root.ca.pem", "../../share/jetbot/certs/private.pem.key", "../../share/jetbot/certs/certificate.pem.crt")
    rospy.loginfo("Starting application loop...")
    rospy.Subscriber("jetbot_camera/raw", Image, callback)
    #iotClient.connect()


if __name__ == '__main__':
    try:
        detect()
        rospy.spin()
    except Exception as e:
        logging.exception("An error occurred")
        raise e
