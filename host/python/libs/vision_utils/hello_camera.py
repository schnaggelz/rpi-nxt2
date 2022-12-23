
import time
import numpy as np
import threading
import picamera2 as picam2

from picamera2 import encoders as picam2_encoders
from picamera2 import outputs as picam2_outputs

from vision_utils import video_sender


sender = video_sender.VideoSender('treich-dt1', 1234)
sender.connect()

camera = picam2.PiCamera2()

lores_size = (320, 240)
main_size = (640, 480)


video_config = camera.create_video_configuration(
    main={"size": main_size, "format": "XRGB8888"},
    lores={"size": lores_size, "format": "YUV420"})

camera.configure(video_config)

#encoder = picam2_encoders.H264Encoder(1000000, repeat=True)
#output = picam2_outputs.CircularOutput()
#encoder.output = [output]

#camera.encoder = encoder
camera.start()
#camera.start_encoder()

while True:
    img = camera.capture_array()
    sender.send(img)
    