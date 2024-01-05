#!/usr/bin/env python3

import asyncio
import json
import os
import threading
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from av import VideoFrame
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

ROOT = '/underlay_ws/src/rae_cvl/rae_cvl/'

class SharedImageBuffer:
    def __init__(self):
        self.frame = None
        self.lock = threading.Lock()

    def update(self, frame):
        with self.lock:
            self.frame = frame

    def get(self):
        with self.lock:
            return self.frame

class ImageSubscriber(Node):
    def __init__(self, image_buffer):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/rae/right/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.image_buffer = image_buffer

    def listener_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_buffer.update(cv_image)
        except CvBridgeError as e:
            self.get_logger().error('CvBridge Error: {}'.format(e))

class OpenCVVideoTrack(VideoStreamTrack):
    def __init__(self, image_buffer):
        super().__init__()
        self.image_buffer = image_buffer

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        frame = self.image_buffer.get()
        if frame is None:
            return await self.recv()  # Wait for the next frame
        video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

async def index(request):
    content = open(os.path.join(ROOT, "index.html"), "r").read()
    return web.Response(content_type="text/html", text=content)

async def javascript(request):
    content = open(os.path.join(ROOT, "client.js"), "r").read()
    return web.Response(content_type="application/javascript", text=content)

async def offer(request, image_buffer):
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    video = OpenCVVideoTrack(image_buffer)
    pc.addTrack(video)
    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps(
            {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        ),
    )

pcs = set()

async def on_shutdown(app):
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

def run_ros_node(image_subscriber):
    rclpy.spin(image_subscriber)

def main():
    image_buffer = SharedImageBuffer()
    rclpy.init()
    image_subscriber = ImageSubscriber(image_buffer)
    ros_thread = threading.Thread(target=run_ros_node, args=(image_subscriber,))
    ros_thread.start()

    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.router.add_get("/", index)
    app.router.add_get("/client.js", javascript)
    app.router.add_post("/offer", lambda request: offer(request, image_buffer))
    web.run_app(app, host="0.0.0.0", port=8080)

    image_subscriber.destroy_node()
    rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()
