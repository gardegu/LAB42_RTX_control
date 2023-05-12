import numpy as np
import cv2
import os
import argparse

def setStereoCameraMode(camera_mode):
    if camera_mode == 'VGA':
        stereo_width  = 1344
        stereo_height = 376
    elif camera_mode == 'HD':
        stereo_width  = 2560
        stereo_height = 720
    elif camera_mode == 'FHD' or camera_mode == 'FHD2':
        stereo_width  = 3840
        stereo_height = 1080
    else:
        sys.exit("Error: Wrong camer mode {}".format(camera_mode))

    out_width     = int(stereo_width/2)
    out_height    = int(stereo_height)

    if camera_mode == 'FHD2':
        ver_offset = int((stereo_height - 1024)/2)
    else:
        ver_offset = 0

    return stereo_width, stereo_height, out_width, out_height, ver_offset


def main(camera_mode, device_num, image_path):
    """Main function"""

    stereo_width, stereo_height, out_width, out_height, ver_offset = \
        setStereoCameraMode(camera_mode)

    # /dev/videoX (X = device_num)
    vid_capture = cv2.VideoCapture(device_num) 
    vid_capture.set(cv2.CAP_PROP_FRAME_WIDTH, stereo_width)
    vid_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, stereo_height)

    # Camera parameters to undistort and rectify images
    cv_file = cv2.FileStorage()

    # Open stereo map file
    mapFile = os.path.join(image_path, 'stereoMap.xml')
    assert os.path.exists(mapFile) != 0, 'Cannot find stereoMap.xml'
    cv_file.open(mapFile, cv2.FileStorage_READ)

    stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
    stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
    stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
    stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()

    while(vid_capture.isOpened()):

        succes, img = vid_capture.read()

        imgL = img[ver_offset:out_height-ver_offset, 0:out_width]
        imgR = img[ver_offset:out_height-ver_offset, out_width:2*out_width]

        # Undistort and rectify images
        imgR = cv2.remap(imgR, stereoMapR_x, stereoMapR_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
        imgL = cv2.remap(imgL, stereoMapL_x, stereoMapL_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

        # Overlay lines
        line_thickness = 1
        for i in range(50, out_height-1, 50):
            cv2.line(imgL, (0, i), (out_width-1, i), (0, 255, 0), thickness=line_thickness)
            cv2.line(imgR, (0, i), (out_width-1, i), (0, 255, 0), thickness=line_thickness)

        # Show the frames
        cv2.imshow("frame right", imgR)
        cv2.imshow("frame left",  imgL)

        # Hit "q" to close the window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release video capture
    vid_capture.release()

    # close windows
    cv2.destroyAllWindows()


if __name__ == '__main__':
    # input argument parser
    parser = argparse.ArgumentParser(description='Stereo ZED camera capture')

    # Path to directory
    parser.add_argument('-p', '--path', type=str, default='./stereo_calib',
        help="Path to the directory storing images for calibration")
    # valid camera_mode: 'FHD', 'FHD2', HD', 'VGA'
    # FHD2: 1920x1024
    parser.add_argument('-m', '--mode',type=str, default='HD',
        help="Camera mode. Valid camera mode: FHD, FHD2, HD, VGA")
    # Device number
    parser.add_argument('-d', '--device', type=int, default=2,
        help="Video device number")
    args = parser.parse_args()

    camera_mode = args.mode
    device_num  = args.device
    image_path  = args.path
    main(camera_mode, device_num, image_path)