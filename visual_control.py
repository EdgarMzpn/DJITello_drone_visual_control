import cv2
import numpy as np
import time
from djitellopy import Tello

# Set up the parameters for SimpleBlobDetector
params = cv2.SimpleBlobDetector_Params()

# Filter by circularity
params.filterByCircularity = True
params.minCircularity = 0.75

# Filter by convexity
params.filterByConvexity = True
params.minConvexity = 0.7

# Filter by area
params.filterByArea = True
params.minArea = 1000

# Filter by inertia
params.filterByInertia = True
params.minInertiaRatio = 0.1

# Create a SimpleBlobDetector object
detector = cv2.SimpleBlobDetector_create(params)

# Define the lower and upper bounds for the colors in HSV
lower_green = (0, 0, 70)
upper_green = (360, 255, 255)

# Create a VideoCapture object to access the webcam
#cap = cv2.VideoCapture(0)

# Check if the webcam is opened successfully
#if not cap.isOpened():
#    print("Error opening the webcam.")
#    exit()

def find_green_circle(frame):
    # Convert the frame from BGR to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask based on the specified color range
    mask = cv2.inRange(hsv_frame, lower_green, upper_green)

    # Apply the mask to the frame
    masked_frame = cv2.bitwise_and(frame, frame, mask=mask)

    # Detect blobs in the masked frame
    keypoints = detector.detect(masked_frame)

    # Draw circles around the detected blobs
    blobs_image = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Draw the center of each detected circle and calculate the area
    height, width, _ = frame.shape

    x_global = 0
    y_global = 0

    area = 0

    x_gain = 0
    y_gain = 0
    z_gain = 0

    for keypoint in keypoints:
        x = int(keypoint.pt[0])
        y = int(keypoint.pt[1])
        cv2.circle(blobs_image, (x, y), 2, (0, 255, 0), -1)

        radius = int(keypoint.size / 2)
        area = np.pi * radius**2

        x_global = x - width//2
        y_global = height//2 - y

    if 0 - x_global < -25 or 0 - x_global > 25:
        x_gain = int(pid_controller(x_global - 0, 0.1, 0, 0))

    if 0 - y_global < -25 or 0 - y_global > 25:
        y_gain = max(min(int(pid_controller(0 + y_global, 0.001, 0, 0)), 15), -15)

    if area < 3000 or area > 3500:
        z_gain = int(pid_controller((4500 - area)//10, 0.1, 0, 0))

    if area == 0:
        z_gain = 0

    return blobs_image, x_gain, y_gain, z_gain

# Variables for pid controller
integral = 0
previous_error = 0

def pid_controller(error,kp,ki,kd):
    global integral
    global previous_error

    # Calculate proportional term
    proportional = kp * error
    
    # Calculate integral term
    integral = integral + (error * ki)
    
    # Calculate derivative term
    derivative = kd * (error - previous_error)
    
    # Calculate control output
    control_output = proportional + integral + derivative
    
    # Output saturation
    control_output = max(min(control_output, 40), -40)

    # Update previous error
    previous_error = error
    
    return control_output

if __name__ == '__main__':
    tl_drone = Tello()
    tl_drone.connect()
    tl_drone.streamon()

    tl_drone.takeoff()

    while True:
        # Read the current frame from the webcam
        frame = tl_drone.get_frame_read().frame

        processed_frame, x, y, z = find_green_circle(frame)

        # a = roll, b = pitch, c = acceleraition, d = yaw
        tl_drone.send_rc_control(0, z, y, x)
        print("yaw: ", x, "\npitch: ", z, "\nacceleration: ", y)

        # Display the camera image and prcessed image
        #cv2.imshow("Drone Cam", frame)
        cv2.imshow("Drone Flight", processed_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release camera resources
    tl_drone.streamoff()
    tl_drone.land()

    cv2.destroyAllWindows()
