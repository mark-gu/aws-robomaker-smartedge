#!/usr/bin/env python3
import torch
import torchvision
import time
import os
import datetime
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import base64
import json
import random
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import PIL.Image
import rospkg

CAFILE = 'root.ca.pem'
KEYFILE = 'private.pem.key'
CERTIFICATEFILE = 'certificate.pem.crt'
ROSAPP = 'jetbot_app'
ENDPOINT = os.environ['IOT_ENDPOINT'].lower()

class DinoDetect:
    UNKNOWN_DINO = 5
    probability_threshold = 0.6
    
    def path(self, filename):
        """
        Creates the full path to the certificate files in the ROS application
        This is needed so the MQTT client can load the certs to authenticate with AWS IoT Core
        """
        rospack = rospkg.RosPack()
        return os.path.join(rospack.get_path(ROSAPP), 'config', filename)
        
    def __init__(self):
        self.setup_data()
        self.setup_cuda_ml_models()
        self.prev_class = None
        self.bridge = CvBridge()
        self.init_ros_pub_sub()
        self.setup_iot_connect()

    def setup_cuda_ml_models(self):
        rospy.loginfo("Starting cuda...")
        self.device = torch.device('cuda')

        rospy.loginfo("Road follow model ...")
        self.model_roadfollow = torchvision.models.resnet18(pretrained=False)
        self.model_roadfollow.fc = torch.nn.Linear(512, 2)
        self.model_roadfollow.load_state_dict(torch.load(self.config['road_following_model']))
        self.model_roadfollow = self.model_roadfollow.to(self.device)
        self.model_roadfollow = self.model_roadfollow.eval().half()

        rospy.loginfo("Dino detection model ...")
        self.model_dinodet = torchvision.models.resnet18(pretrained=False)
        self.model_dinodet.fc = torch.nn.Linear(512, 6)
        self.model_dinodet.load_state_dict(torch.load(self.config['dino_detect_model']))
        self.model_dinodet = self.model_dinodet.to(self.device)
        self.model_dinodet = self.model_dinodet.eval()

    def setup_iot_connect(self):
        self.iotClient = AWSIoTMQTTClient(rospy.param("robot_name").lower())
        self.iotClient.configureEndpoint(rospy.param("iot_endpoint").lower(), int(rospy.param("mqtt_port")))
        self.iotClient.configureCredentials(self.path(CAFILE), self.path(KEYFILE), self.path(CERTIFICATEFILE))
        self.iotClient.configureConnectDisconnectTimeout(10)
        self.iotClient.configureMQTTOperationTimeout(5)
        rospy.loginfo("Connecting to iot core")
        self.iotClient.connect()

    def init_ros_pub_sub(self):
        self.dir_pub = rospy.Publisher('move/cmd_dir', String)
        self.image_sub = rospy.Subscriber("jetbot_camera/raw", Image, self.image_callback)

    def setup_data(self):
        self.device = None
        self.config = {
            "mean_values": [0.485, 0.456, 0.406],
            "std_values": [0.229, 0.224, 0.225],
            "np_value": 255.0,
            "dino_names": [
                "Spinosaurus",
                "Dilophosaurus",
                "Stegosaurus",
                "Triceratodps",
                "Brachiosaurus",
                "Unknown"],
            "road_following_model": "/home/ggc_user/mlmodels/best_steering_model_xy.pth",
            "dino_detect_model": "/home/ggc_user/mlmodels/best_dinodet_model_xy.pth",
            "image_size": [224,224]
        }
        self.settings = {
            "mean_roadfollow": None,
            "std_roadfollow": None
        }

        self.mean = self.config['np_value'] * np.array(self.config['mean_values'])
        self.stdev = self.config['np_value'] * np.array(self.config['std_values'])
        self.normalize = torchvision.transforms.Normalize(self.mean, self.stdev)
        self.mean_roadfollow = torch.Tensor([0.485, 0.456, 0.406]).cuda().half()
        self.std_roadfollow = torch.Tensor([0.229, 0.224, 0.225]).cuda().half()

    def preprocess(self, camera_value_bgr):
        camera_value_rgb = cv2.cvtColor(camera_value_bgr, cv2.COLOR_BGR2RGB)
        camera_value_transpose = camera_value_rgb.transpose((2, 0, 1))
        camera_value_float = torch.from_numpy(camera_value_transpose).float()
        camera_value_norm = self.normalize(camera_value_float)
        camera_value_to_device = camera_value_norm.to(self.device)
        x = camera_value_to_device[None, ...]
        return x

    def preprocess_roadfollow(self, image):
        pil_image = PIL.Image.fromarray(image)
        trans_image = torchvision.transforms.functional.to_tensor(pil_image).to(self.device).half()
        trans_image.sub_(self.mean_roadfollow[:, None, None]).div_(self.std_roadfollow[:, None, None])
        return trans_image[None, ...]

    def find_dino(self, image):
        preprocessed_image = self.preprocess(image)
        y = self.model_dinodet(preprocessed_image)
        y_dino = torch.nn.functional.softmax(y, dim=1)
        topk = y_dino.cpu().topk(1)
        return (e.data.numpy().squeeze().tolist() for e in topk)

    def roadfollow_to_move(self, image, robot_stop):
        move_dir_sent = {}
        move_dir_sent['angle'] = 0

        # If robot is not stopped, find out what angle the robot has to follow.
        if not robot_stop:
            xy = self.model_roadfollow(self.preprocess_roadfollow(image)).detach().float().cpu().numpy().flatten()
            x = xy[0]
            y = (0.5 - xy[1]) / 2.0
            self.angle = np.arctan2(x, y)
        move_dir_sent['stop_robot'] = robot_stop

        self.dir_pub.publish(json.dumps(move_dir_sent))

    def image_callback(self, data):
        robot_stop = False
        img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        probability, current_class = self.find_dino(img) 

        # Send data/stop robot only if you see a dino that you haven't before
        if probability > DinoDetect.probability_threshold and current_class != self.prev_class:
            rospy.loginfo("Found %s...", self.config['dino_names'][current_class])
            message = {
                "dinosaur": self.config['dino_names'][current_class],
                "confidence": probability,
                "image": base64.b64encode(Image.fromarray(img))
            }

            # Send the data to the iot cloud when you find a dino
            self.iotClient.publish(rospy.param("robot_name").lower()+"/dinos", json.dumps(message), 1)
            self.prev_class = current_class

            # Stop the robot everytime you see a new robot
            robot_stop = True

        # Send image to roadfollowing after
        self.roadfollow_to_move(img, robot_stop)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('dino_detect', log_level=rospy.DEBUG)
    dino = DinoDetect()
    dino.main()