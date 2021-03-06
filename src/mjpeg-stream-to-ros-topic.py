#!/usr/bin/env python
# coding=UTF-8

import cv2
import urllib 
import numpy as np

import roslib
roslib.load_manifest('video_stream_opencv')
import signal
import sys
import time
import rospy
import yaml
import argparse

from cv_bridge import CvBridge, CvBridgeError
#from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo

print(sys.version)
print(sys.argv)

#args.topic_name     = 'mjpeg_publisher'
#args.stream_url     = 'http://iris.not.iac.es/axis-cgi/mjpg/video.cgi?resolution=320x240'
#args.jpeg_quality   = 40
#args.est_image_size = 409600
#args.show_gui       = False
#args.verbose        = True
#args.caminfo_file   = '/home/biagio/.ros/camera_info/vivotek_IB8168_C.yaml'

parser = argparse.ArgumentParser(description='Given a MJPEG HTTP stream, this node publishes a CompressedImage (and its CameraInfo) topics.')
parser.add_argument('stream_url', help='The MJPEG URL', metavar='http://vivotek-0:8080/video3.mjpg',
                              default='http://vivotek-0:8080/video.mjpg')
parser.add_argument('topic_name', help='The output topic', metavar='image',
                              default='vivotek_0')
parser.add_argument('-q', help='The jpeg quality for an optional republish', dest='jpeg_quality', type=int,
                        metavar='40')
parser.add_argument('est_image_size', help='The estimated image size, to optimize recv speed', type=int,
                        metavar='409600')
parser.add_argument('--show_gui', help='Show the images that are being received', dest='show_gui', type=bool,
                        metavar=False, default=False)
parser.add_argument('-v', help='Be more verbose', dest='verbose', type=bool,
                        metavar=False, default=False)
parser.add_argument('-c', help='Camera info file', metavar='~/.ros/camera_info/camera.yaml', dest='caminfo_file', default='')

args, unknown = parser.parse_known_args()

print("URL: ", args.stream_url, "Output topic: ", args.topic_name, "Est. image size: ", args.est_image_size, "Camera info file: ", args.caminfo_file, "Jpeg quality: ", args.jpeg_quality, "Show GUI: ", args.show_gui, "Verbose: ", args.verbose)


def parse_calibration_yaml(calib_file):
    with file(calib_file, 'r') as f:
        params = yaml.load(f)

    cam_info = CameraInfo()
    cam_info.header.frame_id = params['camera_name']
    cam_info.height = params['image_height']
    cam_info.width = params['image_width']
    cam_info.distortion_model = params['distortion_model']
    cam_info.K = params['camera_matrix']['data']
    cam_info.D = params['distortion_coefficients']['data']
    cam_info.R = params['rectification_matrix']['data']
    cam_info.P = params['projection_matrix']['data']

    return cam_info


stream = urllib.urlopen(args.stream_url)

rospy.init_node('mjpeg_stream_to_ros_topic', anonymous=True)
mjpeg_publisher      = rospy.Publisher (args.topic_name + '/compressed'		, CompressedImage, queue_size = 1)
if args.jpeg_quality > 0:
	low_qual_republisher = rospy.Publisher (args.topic_name + '_low_qual' + '/compressed', CompressedImage, queue_size = 1)

if args.caminfo_file != '':
	cam_pub  = rospy.Publisher("camera_info", CameraInfo, queue_size = 1)
	cam_info = parse_calibration_yaml(args.caminfo_file)
else:
	cam_pub  = None

def signal_handler(sig, frame):
        print('Ctrl+C pressed, exiting...')
        sys.exit(0)

def jpeg_publisher(data, publisher, cam_pub = None):
        stamp = rospy.Time.now()
	if cam_pub is not None:
		cam_info.header.stamp = stamp
		# publish the camera info messages first
		cam_pub.publish(cam_info)
        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.format = "jpeg"
        #msg.format = "rgb8; jpeg compressed bgr8"
        #msg.data = np.array(cv2.imencode('.jpg', self.last_img)[1]).tostring()
        msg.data = data.tostring()
        # Publish new image
        publisher.publish(msg)



signal.signal(signal.SIGINT, signal_handler)
print('Press Ctrl+C to quit')

bytes = ''
a = b = -1
while True:
    bytes += stream.read(args.est_image_size)
    if a == -1:
	a = bytes.find('\xff\xd8')
    if b == -1:
	b = bytes.find('\xff\xd9')
    if a != -1 and b != -1:
        jpg = bytes[a:b+2]
        bytes = bytes[b+2:]
	#print("----------------------------------------------------------", a, b, len(bytes))

	a = b = -1

	numpy_data = np.fromstring(jpg, dtype=np.uint8)
	if args.show_gui:
		i = cv2.imdecode(numpy_data, cv2.IMREAD_COLOR)
		cv2.imshow('img', i)
		if cv2.waitKey(1) == 27:
			exit(0)   

	if args.jpeg_quality > 0:
		i = cv2.imdecode(numpy_data, cv2.IMREAD_COLOR)
		retval, jpeg_data = cv2.imencode('.jpg', i, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
		if args.verbose:
			print("Successful recompression: {} - orig jpeg: {} - recompressed: {}".format(retval, numpy_data.size, jpeg_data.size))
		if retval:
			jpeg_publisher(jpeg_data, low_qual_republisher)

        jpeg_publisher(numpy_data, mjpeg_publisher, cam_pub)
