#!/usr/bin/env python
"""

ROS Node for object detection and gaze tracking
Publishes to ALMA

See the julia repo top level README for more info

"""
import pyfreenect2
import cv2
import signal
import numpy as np
import os
from gaze_tracking import GazeTracking

import rospy
import cv_bridge
from std_msgs.msg import String
from sensor_msgs.msg import Image

rospy.init_node('vision')
pub = rospy.Publisher('alma_in', String, queue_size=10)
videoPub = rospy.Publisher('/robot/xdisplay', Image, latch=True)

# 'path to yolo config file'
# download https://github.com/arunponnusamy/object-detection-opencv/blob/master/yolov3.cfg
CONFIG = 'yolov3.cfg'

# 'path to text file containing class names'
# download https://github.com/arunponnusamy/object-detection-opencv/blob/master/yolov3.txt
CLASSES = 'yolov3.txt'

# 'path to yolo pre-trained weights'
##### ATTENTION: This is too big for GitHub, so you will have to download it separately (see the README). #####
# wget https://pjreddie.com/media/files/yolov3.weights
WEIGHTS = 'yolov3.weights'

# read class names from text file
classes = None
__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
with open(os.path.join(__location__, CLASSES), 'r') as f:
    classes = [line.strip() for line in f.readlines()]

scale = 0.00392
conf_threshold = 0.5
nms_threshold = 0.4

# generate different colors for different classes
COLORS = np.random.uniform(0, 255, size=(len(classes), 3))

# function to get the output layer names
# in the architecture
def get_output_layers(net):
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    return output_layers


# function to draw bounding box on the detected object with class name
def draw_bounding_box(img, class_id, confidence, x, y, x_plus_w, y_plus_h):
    label = str(classes[class_id]) + " " + str(round(confidence, 3))
    color = COLORS[class_id]
    # Sometimes rectangle calls will give you errors even if some documentation says you're right
    # The signatures changed between OpenCV versions (WHY??)
    # Errors about argument numbers means you're not using the right method for the right version
    cv2.rectangle(img, (x, y, x_plus_w - x, y_plus_h - y), color, 2)
    cv2.putText(img, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def process_image(image, depth, net, gaze):
    width = image.shape[1]
    height = image.shape[0]

    # create input blob
    blob = cv2.dnn.blobFromImage(image, scale, (416, 416), (0, 0, 0), True, crop=False)
    # set input blob for the network
    net.setInput(blob)

    # run inference through the network
    # and gather predictions from output layers
    outs = net.forward(get_output_layers(net))

    # initialization
    class_ids = []
    confidences = []
    boxes = []
    # for each detection from each output layer
    # get the confidence, class id, bounding box params
    # and ignore weak detections (confidence < 0.5)
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                x = center_x - w / 2
                y = center_y - h / 2
                class_ids.append(class_id)
                confidences.append(float(confidence))
                boxes.append([x, y, w, h])

    # apply non-max suppression
    indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)

    # go through the detections remaining
    # after nms and draw bounding box
    ann = cv2.cvtColor(depth, cv2.COLOR_BGRA2BGR)
    for i in indices:
        i = i[0]
        box = boxes[i]
        x = max(0, box[0])
        y = max(0, box[1])
        w = max(0, box[2])
        h = max(0, box[3])
        """
        # Trying to do something with depth data, but it seems like gibberish
        # There's an issue open on the pyfreenect2 GitHub
        print(depth.shape)
        d = depth[int((y + h / 2) * 424 / 1080)][int((x + w / 2) * 512 / 1920)]
        print("test")
        print(d)
        sum = [0, 0, 0, 0]
        for j in range(int((x + w / 2) * 512 / 1920), int((x + w / 2) * 512 / 1920) + 10):
            for k in range(int((y + h / 2) * 424 / 1080), int((y + h / 2) * 424 / 1080) + 10):
                sum = np.add(depth[k][j], sum)
        sum = sum / 100
        print(sum)
        print(image.shape)
        print(image[int((y + h / 2))][int((x + w / 2))])
        distance = d[3] / 255
        """

        # Gaze Detection
        if str(classes[class_ids[i]]) == "person":
            # Run gaze detection only on the bounding box region of people
            # Somewhat redundant, as the gaze detection does face recognition as well
            # Seems necessary though, as it won't detect multiple people, and it needs the face data to determine direction
            gaze.refresh(image[y:y + h, x:x + w])

            # Place green crosses on eyes
            image[y:y + h, x:x + w] = gaze.annotated_frame()

            # This is weird
            # Even if pupils_located is a success, the ratio methods subtract 10 from a denominator somewhere
            # So sometimes there's a divide by 0 error
            # This try catch is just to stop the program from dying, but sometimes it still does
            # It's dying when the video feeds freeze and go black and white
            if gaze.pupils_located:
                try:
                    cv2.putText(image, "pupil x: " + str(round(gaze.horizontal_ratio(), 3)) + " y: " + str(
                        round(gaze.vertical_ratio(), 3)), (int(x + w - 200), y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                COLORS[class_ids[i]], 2)
                except ZeroDivisionError:
                    pass

        # ALMA Communication
        # saw(label, confidence (50-100), center_X, center_Y, width, height, time)
        # Note that time gets added by ALMA because we use obs (see ALMA documentation)
        # Still no depth
        pub.publish(
            "obs saw(" + str(classes[class_ids[i]]) + "," + str(int(100*confidences[i])) + "," + str(int(x)) + "," + str(
                int(y)) + "," + str(int(w)) + "," + str(int(h)) + ").\n")

        # Draws bounding box on color image
        # Math with passing parameters probably not optimal, but just some addition so not too time inefficient
        draw_bounding_box(image, class_ids[i], confidences[i], int(x), int(y), int(x + w), int(y + h))

        # Draws small point at the center of every box on the depth image
        # These points don't seem to match up; depth sensor and color sensor aren't the same aspect ratio or FOV
        # Some kinect drivers have the ability to flatten it out or get real space coordinates
        draw_bounding_box(ann, class_ids[i], confidences[i], int((x + w / 2) * 512 / 1920),
                          int((y + h / 2) * 424 / 1080), 5 + int((x + w / 2) * 512 / 1920),
                          5 + int((y + h / 2) * 424 / 1080))

    # display color and depth output on screen
    out_image_name = "color"  # + str(index)
    cv2.imshow(out_image_name, image)
    cv2.imshow("depth", ann)

    #Send color output to Baxter screen
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(cv2.resize(image, (1024, 600)), encoding="bgr8")
    videoPub.publish(msg)

    # wait until any key is pressed
    # cv2.waitKey()
    # save output image to disk
    # cv2.imwrite("out/" + out_image_name + ".jpg", image)


serialNumber = pyfreenect2.getDefaultDeviceSerialNumber()
kinect = pyfreenect2.Freenect2Device(serialNumber)

# Set up signal handler
shutdown = False


def sigint_handler(signum, frame):
    print("Got SIGINT, shutting down...")
    global shutdown
    shutdown = True


signal.signal(signal.SIGINT, sigint_handler)

# Set up frame listener
frameListener = pyfreenect2.SyncMultiFrameListener(pyfreenect2.Frame.COLOR,
                                                   pyfreenect2.Frame.IR,
                                                   pyfreenect2.Frame.DEPTH)

# print(frameListener)
kinect.setColorFrameListener(frameListener)
kinect.setIrAndDepthFrameListener(frameListener)
kinect.setDeepConfiguration(
    pyfreenect2.DeepConfig(MinDepth=.5, MaxDepth=10, EnableBilateralFilter=True, EnableEdgeAwareFilter=True))

# Start recording
kinect.start()

# What's a registration?
# print(kinect.ir_camera_params)

registration = pyfreenect2.Registration(kinect.ir_camera_params, kinect.color_camera_params)
# registration = pyfreenect2.Registration(kinect.color_camera_params, kinect.ir_camera_params)
# registration = pyfreenect2.Registration()

# Main loop

# read pre-trained model and config file
net = cv2.dnn.readNet(os.path.join(__location__, WEIGHTS), os.path.join(__location__, CONFIG))
# Init gaze tracking
gaze = GazeTracking()

while not shutdown:
    frames = frameListener.waitForNewFrame()
    rgbFrame = frames.getFrame(pyfreenect2.Frame.COLOR)
    # IR stuff seems to be stubbed in pyfreenect2. Grrrr
    # irFrame = frames.getFrame(pyfreenect2.Frame.IR)
    depthFrame = frames.getFrame(pyfreenect2.Frame.DEPTH)
    rgb_frame = rgbFrame.getRGBData()
    bgr_frame = rgb_frame.copy()
    bgr_frame[:, :, 0] = rgb_frame[:, :, 2]
    bgr_frame[:, :, 2] = rgb_frame[:, :, 0]

    depth_frame = depthFrame.getDepthData()
    # depth_frame = frames.getFrame(pyfreenect2.Frame.DEPTH).getDepthData()
    # bgr_frame_resize = scipy.misc.imresize(bgr_frame, size = .5)
    # depth_frame_resize = scipy.misc.imresize(depth_frame, size=.5)

    # Remove alpha channel
    bgr_frame_new = cv2.cvtColor(bgr_frame, cv2.COLOR_BGRA2BGR)

    # depth_frame = cv2.cvtColor(depth_frame, cv2.COLOR_BGR2GRAY)
    # depth_frame = cv2.cvtColor(depth_frame, cv2.COLOR_GRAY2BGR)

    # process_image(cv2.resize(bgr_frame_new,(1024,600)))

    # Do the detection
    process_image(bgr_frame_new, depth_frame, net, gaze)

    cv2.waitKey(20)
    frameListener.release(frames)

kinect.stop()
kinect.close()
