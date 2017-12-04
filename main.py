#!/usr/bin/env python

import numpy as np
import cv2
import cv2.cv as cv
from threading import Thread
import time
import serial

last_received = ''

actiongroupend = True

meetleft= False
meetright = False
servorunning = False

def receiving(serial_port):
    global last_received
    global actiongroupend
    buffer = ''
    while True:
        buffer += serial_port.read_all()
        if "UU" in buffer:
            lines = buffer.split("UU")  # Guaranteed to have at least 2 entries
            if lines[-2]: last_received = lines[-2]
            # If the Arduino sends lots of empty lines, you'll lose the last
            # filled line, so you could make the above statement conditional
            # like so: if lines[-2]: last_received = lines[-2]
            buffer = lines[-1]
            actiongroupend = True



class SerialData(object):
    def __init__(self,comport):
        try:
            self.serial_port = serial.Serial(comport,9600)
        except serial.serialutil.SerialException:
            # no serial connection
            self.serial_port = None
        else:
            Thread(target=receiving, args=(self.serial_port,)).start()

    def send(self, data):
        self.serial_port.write(data)

    def __del__(self):
        if self.serial_port is not None:
            self.serial_port.close()

def detect(img, cascade):
    rects = cascade.detectMultiScale(img, scaleFactor=1.3, minNeighbors=4, minSize=(30, 30), flags = cv.CV_HAAR_SCALE_IMAGE)
    if len(rects) == 0:
        return []
    rects[:,2:] += rects[:,:2]
    return rects

def draw_rects(img, rects, color):
    for x1, y1, x2, y2 in rects:
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

def draw_smiles(img, rects, color):
    h,w,c = img.shape
    for x1, y1, x2, y2 in rects:
        cv2.rectangle(img, (y1, h-x1/2), (y2, h-x2/2), color, 2)

def runActionGroup(numOfAction, Times, serialpo):
    FRAME_HEADER = 0x55
    CMD_ACTION_GROUP_RUN = 0x06
    command = bytearray([FRAME_HEADER,FRAME_HEADER,0x05,CMD_ACTION_GROUP_RUN,numOfAction,Times,0x00])
    serialpo.send(command)

def stopActionGroup(serialpo):
    FRAME_HEADER = 0x55
    CMD_ACTION_GROUP_STOP = 0x07
    command = bytearray([FRAME_HEADER,FRAME_HEADER,2,CMD_ACTION_GROUP_STOP])
    serialpo.send(command)

def signalprocess(data):
    meetleft = bool(data/8)
    data = data%8
    meetright = bool(data/4)
    data = data%4
    servorunning = bool(data/2)
    return meetleft,meetright,servorunning

if __name__ == '__main__':
    import sys, getopt

    STAND = 0
    FORWARD = 1
    BACKWARD = 2
    LEFT = 3
    RIGHT = 4
    FORWARDONCE = 7
    BACKWARDONCE = 8

    width = 640
    center = 320
    centertol = 0.1
    height = 480

    args, video_src = getopt.getopt(sys.argv[1:], '', ['cascade=', 'nested-cascade='])
    args = dict(args)
    cascade_fn = args.get('--cascade', "data/haarcascades/haarcascade_frontalface_alt.xml")
    smile_fn = args.get('--smile',"data/haarcascades/haarcascade_smile.xml")

    cascade = cv2.CascadeClassifier(cascade_fn)
    smile = cv2.CascadeClassifier(smile_fn)

    #cam = create_capture(video_src, fallback='synth:bg=../cpp/lena.jpg:noise=0.05')
    cap = cv2.VideoCapture(0)
    #cap.set(cv2.cv.CV_CAP_PROP_FOURCC,cv2.cv.CV_FOURCC('M', 'J', 'P', 'G') )
    #cap.set(cv2.cv.CV_CAP_PROP_EXPOSURE, -6)
    cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, height)

    s = SerialData('com7')

    while True:
        smileBool = False
        pos = -1
        ret, img = cap.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        rects = detect(gray, cascade)
        vis = img.copy()
        facesizes = []
        facesmiles = []
        facepositions = []
        for x1, y1, x2, y2 in rects:
            facesizes.append(abs((x2-x1)*(y2-y1)))
            facepositions.append((x1+x2)/2)
            roi = gray[y1:y2, x1:x2]
            smilerects = smile.detectMultiScale(
                roi.copy(),
                scaleFactor=1.7,
                minNeighbors=22,
                minSize=(25, 25),
                flags=cv2.cv.CV_HAAR_SCALE_IMAGE
            )
            appended = False
            for smilerect in smilerects:
                facesmiles.append(True)
                appended = True
                break
            if not appended:
                facesmiles.append(False)
        tempsize = -1
        temppos = -1
        if True not in facesmiles:
            for a in range(len(facesizes)):
                if facesizes[a] > tempsize:
                    tempsize = facesizes[a]
                    temppos= a
            if tempsize > -1:
                print tempsize
                pos = facepositions[temppos]
        else:
            smileBool = True
            for a in range(len(facesizes)):
                if facesmiles[a]:
                    if facesizes[a] > tempsize:
                        tempsize = facesizes[a]
                        temppos = a
            pos = facepositions[temppos]
        if meetleft and meetright:
            if (actiongroupend):
                actiongroupend = False
                runActionGroup(2, 1, s)
        elif meetleft:
            if (actiongroupend):
                actiongroupend = False
                runActionGroup(2, 1, s)
        elif meetright:
            if (actiongroupend):
                actiongroupend = False
                runActionGroup(2, 1, s)
        else:
            if pos != -1:
                if pos > center*(1+centertol):
                    print "right"
                    if (actiongroupend):
                        actiongroupend = False
                        runActionGroup(4, 1, s)
                elif pos < center*(1-centertol):
                    print "left"
                    if (actiongroupend):
                        actiongroupend = False
                        runActionGroup(3, 1, s)
                else:
                    print "forward"
                    if(actiongroupend and tempsize < 100000):
                        actiongroupend = False
                        runActionGroup(1,1,s)
                if smileBool:
                    print "smiling"
            else:
                if (actiongroupend):
                    actiongroupend = False
                    runActionGroup(0, 1, s)

        # draw_rects(vis, rects, (0, 255, 0))
        # for x1, y1, x2, y2 in rects:
        #     roi = gray[y1:y2, x1:x2]
        #     vis_roi = vis[y1:y2, x1:x2]
        #     smilerects = smile.detectMultiScale(
        #     roi.copy(),
        #     scaleFactor= 1.7,
        #     minNeighbors=22,
        #     minSize=(25, 25),
        #     flags=cv2.cv.CV_HAAR_SCALE_IMAGE
        #     )
        #     draw_smiles(vis_roi, smilerects, (0,0,255))
        # cv2.imshow('facedetect', vis)

        if 0xFF & cv2.waitKey(5) == 27:
            break
    cv2.destroyAllWindows()
