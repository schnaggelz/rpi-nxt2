
import time
import numpy as np
import threading
import picamera2 as picam2

from picamera2 import encoders as picam2_encoders
from picamera2 import outputs as picam2_outputs

import video_sender as vs
import fps_calcularor as fps


sender = vs.VideoSender('192.168.242.163', 4243)
sender.connect()

camera = picam2.Picamera2()

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

fps = fps.FpsCalculator()

counter = 0
while True:
    img = camera.capture_array()
    sender.send(img)
    fps_val = fps()
    if counter % 100 == 0:
        print("currently sending at {} fps".format(fps_val))
    counter += 1