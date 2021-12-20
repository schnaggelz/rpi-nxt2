import cv2
import numpy as np

import camera_source as cam


class CubeColors:
    COLORS = {
        'lower_red': ([0, 5, 5], [5, 255, 255]),
        'orange': ([6, 195, 195], [15, 255, 255]),
        'yellow': ([21, 145, 195], [35, 255, 255]),
        'green': ([41, 95, 155], [75, 255, 255]),
        'blue': ([76, 75, 75], [135, 255, 255]),
        'purple': ([136, 165, 100], [135, 255, 255]),
        'red': ([166, 100, 150], [180, 255, 255]),
        'white': ([0, 0, 180], [255, 75, 255])
    }

    @staticmethod
    def range(color):
        r = []
        for key, value in CubeColors.COLORS:
            if key == color:
                r.append(value)
        return r

    @staticmethod
    def ranges():
        r = {}
        for key, value in CubeColors.COLORS:
            if key not in r:
                r[key].append(value)
        return r


if __name__ == '__main__':
    address = "/dev/video0"
    cs = cam.CameraSource(address, 1280, 720)
    cs.open()

    # Create a window named trackbars.
    cv2.namedWindow("Trackbars")

    # A required callback method that goes into the trackbar function
    def nothing(x):
        pass


    cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
    cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)
    cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

    while True:

        # Start reading the webcam feed frame by frame.
        frame = cs.read()
        if frame is None:
            break

        # Flip the frame horizontally (Not required)
        frame = cv2.flip(frame, 1)

        # Convert the BGR image to HSV image.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Get the new values of the trackbar in real time as the user changes them
        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
        u_v = cv2.getTrackbarPos("U - V", "Trackbars")

        # Set the lower and upper HSV range according to the value selected by the trackbar
        lower_range = np.array([l_h, l_s, l_v])
        upper_range = np.array([u_h, u_s, u_v])

        # Filter the image and get the binary mask, where white represents your target color
        mask = cv2.inRange(hsv, lower_range, upper_range)

        # Visualize the real part of the target color (optional)
        res = cv2.bitwise_and(frame, frame, mask=mask)

        # Converting the binary mask to 3 channel image, this is just so we can stack it with the others
        mask_3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # Stack the mask, original frame and the filtered result
        stacked = np.hstack((mask_3, frame, res))

        # Show this stacked frame at 40% of the size.
        cv2.imshow('Trackbars', cv2.resize(stacked, None, fx=0.4, fy=0.4))

        if cs.exited():
            break
