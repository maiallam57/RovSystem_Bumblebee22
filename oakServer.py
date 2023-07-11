#kill -9 $(ps -A | grep python | awk '{print $1}')
import sys
sys.path.append('/home/rov/.local/lib/python3.6/site-packages')
import cv2
import depthai as dai
import numpy as np
import time
from vidgear.gears import NetGear

# define various tweak flags
options = {"bidirectional_mode": True,"max_retries": 10000, "jpeg_compression": False}

server = NetGear(
    address="192.168.33.100",
    port=5454,
    protocol="tcp",
    pattern=1,
    logging=True,
    **options
)

class CameraDefinition:
    def __init__(self) -> None:
        # Create pipeline
        self.pipeline = dai.Pipeline()
        self.camRgb = None
        self.monoRight = None
        self.monoLeft = None
        self.depth = None
        self.xoutRgb = None
        self.xoutDepth = None
        self.xoutDesparity = None
        self.xoutRight = None

        self.cameraRgbNode()
        self.monoRightNode()
        self.monoLeftNode()
        self.depthNode()
        self.xoutRgbNode()
        self.xoutDepthNode()
        self.xoutRightNode()

    def cameraRgbNode(self) -> None:
        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        # Properties
        self.camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
        self.camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        self.camRgb.setPreviewSize(640, 400)
        self.camRgb.setFps(50)
        self.camRgb.setInterleaved(False)
    
    def monoRightNode(self) -> None:
        self.monoRight = self.pipeline.create(dai.node.MonoCamera)
        self.monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        self.monoRight.setFps(50)

    def monoLeftNode(self) -> None:
        self.monoLeft = self.pipeline.create(dai.node.MonoCamera)
        self.monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        #self.monoLeft.setFps(50)

    def depthNode(self) -> None:
        # Closer-in minimum depth, disparity range is doubled (from 95 to 190):
        extended_disparity = False
        # Better accuracy for longer distance, fractional disparity 32-levels:
        subpixel = True
        # Better handling for occlusions:
        lr_check = True
        # Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
        self.depth = self.pipeline.create(dai.node.StereoDepth)
        self.depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        # Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
        self.depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        self.depth.setLeftRightCheck(lr_check)
        self.depth.setExtendedDisparity(extended_disparity)
        self.depth.setSubpixel(subpixel)

        # depth configuration
        config = self.depth.initialConfig.get()
        config.postProcessing.speckleFilter.enable = False
        config.postProcessing.speckleFilter.speckleRange = 50
        config.postProcessing.temporalFilter.enable = True
        config.postProcessing.spatialFilter.enable = True
        config.postProcessing.spatialFilter.holeFillingRadius = 2
        config.postProcessing.spatialFilter.numIterations = 1
        config.postProcessing.thresholdFilter.minRange = 200
        config.postProcessing.thresholdFilter.maxRange = 15000
        config.postProcessing.decimationFilter.decimationFactor = 1
        self.depth.initialConfig.set(config)

    def xoutRgbNode(self) -> None:
        # xoutRgb -> for rgb frame output
        self.xoutRgb = self.pipeline.create(dai.node.XLinkOut)
        self.xoutRgb.setStreamName("rgb")
        # Linking rgb Camera
        self.camRgb.preview.link(self.xoutRgb.input)

    def xoutDepthNode(self) -> None:
        self.monoLeft.out.link(self.depth.left)
        self.monoRight.out.link(self.depth.right)

        self.xoutDepth = self.pipeline.create(dai.node.XLinkOut)
        self.xoutDepth.setStreamName("depth")
        self.depth.depth.link(self.xoutDepth.input)

        self.xoutDesparity = self.pipeline.create(dai.node.XLinkOut)
        self.xoutDesparity.setStreamName("disparity")
        self.depth.disparity.link(self.xoutDesparity.input)
    
    def xoutRightNode(self) -> None:
        self.xoutRight = self.pipeline.create(dai.node.XLinkOut)
        self.xoutRight.setStreamName("right")
        self.monoRight.out.link(self.xoutRight.input)

    def getPipeline(self) -> None:
        return self.pipeline
    
    def getDepthMaxDesparity(self) -> None:
        return self.depth.initialConfig.getMaxDisparity()
    
    def normalize(self, img):
        max_value = img.max()
        img_float = np.float32(img)
        img_float /= max_value
        img_float*255
        return max_value, np.uint8(img_float)

oak = CameraDefinition()
pipeLine = oak.getPipeline()
maxDesparity = oak.getDepthMaxDesparity()

# Connect to device and start pipeline
with dai.Device(pipeLine) as device:
    print('Connected cameras: ', device.getConnectedCameras())
    print('Usb speed: ', device.getUsbSpeed().name)

    # Output queue will be used to get the rgb frames from the output defined in class
    qRgb = device.getOutputQueue(name="rgb",  maxSize=4, blocking=False)
    qDisparity = device.getOutputQueue(name="disparity",  maxSize=4, blocking=False)
    qDepth = device.getOutputQueue(name="depth",  maxSize=4, blocking=False)
    qRight = device.getOutputQueue(name="right",  maxSize=4, blocking=False)
    inDisparity, inDepth, inRight = None, None, None
    merged = np.ones((400, 640, 3), dtype=np.uint16)
    switch = "s"

    while True:
        inRgb = qRgb.get()

        if inDisparity is None:
            inDisparity = qDisparity.tryGet()

        if inDepth is None:
            inDepth = qDepth.tryGet()

        if inRight is None:
            inRight = qRight.tryGet()

        if inDisparity is not None and inDepth is not None and inRight is not None:
            depth = inDepth.getFrame()
            desparity = np.uint16((inDisparity.getFrame() * (255 / maxDesparity)).astype(np.uint8))
            right = np.uint16(inRight.getFrame())
            merged = cv2.merge([right, desparity, depth])
            inDisparity, inDepth, inRight = None, None, None
 
        rgbFrame = inRgb.getCvFrame()
        fullFrame = np.hstack([np.uint16(rgbFrame), merged])
        # send frame
        if switch == "s" :
            switch = server.send(rgbFrame, message="S")
        elif switch == "d":
            switch = server.send(merged, message="D")
        elif switch == "f":
            switch = server.send(np.hstack([np.uint16(rgbFrame), merged]), message="F")

        if cv2.waitKey(1) == ord('q'):
            break

# safely close server
server.close()


