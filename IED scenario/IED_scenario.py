import multiprocessing as mp
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import socket
import time
import math
HOST = "192.168.1.3" # The remote host
PORT = 30002 # The same port as used by the server
print ("Starting Program")
count = 0
a = str(0.1) # Set max acceleration value for UR16e
v=str(0.1)   # Set max Velocity value of UR16e
p_home = [0.3,-0.3,0.15]            # x , y , z coordinates of end effector relative to base frame at home position
r_home = [0,1.57,0]              # row, pitch, yaw rotation of end effector relative to base frame at home position
p_camera = [0.190,-0.2,0.660]       # x , y , z coordinates of realsense camera relative to base frame at home position (must change if camera posiiton moved)
p_endeffector = [0.2,-0.230,0.6]    # x , y , z coordinates of end effector relative to base frame (Should not be changed unless you want to change pinhole point of endeffector rotation
p_store =[0,0]
test = 0
forward=p_home[0]

def click_event(event, x, y, flags, params):
 
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
 
        # displaying the coordinates
        # on the Shell
        #print(x, ' ', y)
        user_target_point = (x,y)
        #print(user_target_point)
        real_points = []
        point = user_target_point
        depth = depth_frame.as_depth_frame().get_distance(point[0], point[1])
        real_point = rs.rs2_deproject_pixel_to_point(intrin, [point[0], point[1]], depth)
        #print(real_point)
        # displaying the coordinates
        # on the image window
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(color_image, str(x) + ',' +
                    str(y), (x,y), font,
                    1, (255, 0, 0), 2)
        cv2.imshow('image', color_image)
        #print("---Function---")
        #print(real_point)
        #print("---end function---")
        q.put(real_point)
        return real_point

def foo(q):
    p1=p_home
    row1=str(r_home[0])
    pitch1=str(r_home[1])
    yaw1=str(r_home[2])

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    time.sleep(0.5)
    print ("Set output 1 and 2 high")
    string1=("set_digital_out(1,True)" + "\n")
    s.send (string1.encode())
    time.sleep(1)
    string2=("set_digital_out(2,True)" + "\n")
    s.send (string2.encode())
    time.sleep(1)
    print("Starting slew to home position")
    string6=('movel(p[' + str(p_home[0]) +',' + str(p_home[1]) + ',' + str(p_home[2]) + ',' + row1 + ',' + pitch1 +',' + yaw1 + '],a=' + a + ',v=' + v + ')\n')
    s.send (string6.encode())
    time.sleep(5)
    print("Click a target to begin")
    while True:
        if not q.empty():
            p1 = q.get()
            print("#####################")
            print("The message is:")
            print(p1)
            print("end message")
            ##Begin calculations
            print("\nCalculating rotation angle of EF to point 1")
            x_d= p1[2] + p_camera[0] - p_endeffector[0]             #obtain x, y and z distances between point 1 and end effector
            y_d = p_camera[1] - p1[0] - p_endeffector[1]
            z_d = p_camera[2] - p_endeffector[2] - p1[1]
            print(x_d)
            print(y_d)
            print(z_d)
            theta= math.atan(z_d / x_d)                             #obtain pitch angle: thea 
            alpha = math.atan(y_d / x_d)                            #obtain yaw angle: alpha
            print("Rotation angles with reference to home")
            #Row, pitch and yaw values to send to UR16e
            pitch1 = float(pitch1) - theta
            yaw1 = float(yaw1) + alpha
            row1 = -yaw1

            if abs(p1[0] - p_store[0]) < 0.3 and abs(p1[1] - p_store[1] < 0.3):
                forward = forward + 0.5
            else:
                forward = p_home[0]
        

            print("Pitch :")
            print(pitch1)
            print("Yaw :")
            print(yaw1)
            print("Row :")
            print(row1)
            print("#####################")
            print ("Robot starts Moving to a positions based on pose positions")
            print(p1)
            string6=('movel(p[' + str(forward) +',' + str(p_home[1]) + ',' + str(p_home[2]) + ',' + str(row1) + ',' + str(pitch1) +',' + str(yaw1) + '],a=' + a + ',v=' + v + ')\n')
            s.send (string6.encode())
            time.sleep(1)
            print("done")


            print("done")
            #print ("Set output 1 and 2 low")
            #string9=("set_digital_out(1,False)" + "\n")
            #s.send (string9.encode())
            #time.sleep(0.1)
            #string10=("set_digital_out(2,False)" + "\n")
            #s.send (string10.encode())
            #time.sleep(0.1)
            #count = count + 1
            #print ("The count is:", count)
            #print ("Program finish")
            #time.sleep(1)
            #data = s.recv(1024)
            #s.close()
            #print ("Received", repr(data))
            #print ("Status data received from robot")
            #print("Ending Slew cycle")
            p_store[0] = p1[0]
            p_store[1] = p1[1]


if __name__ == '__main__':
    mp.set_start_method('spawn')
    q = mp.Queue()
    #q.put('hello')
    p = mp.Process(target=foo, args=(q,))
    p.start()

    user_target = (320,240)
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))



    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            intrin = depth_frame.profile.as_video_stream_profile().intrinsics

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape
            images = np.hstack((color_image, depth_colormap))

            cv2.namedWindow('image', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('image', color_image)
            real_point = cv2.setMouseCallback('image', click_event)
            cv2.waitKey(1)


    finally:

        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()





