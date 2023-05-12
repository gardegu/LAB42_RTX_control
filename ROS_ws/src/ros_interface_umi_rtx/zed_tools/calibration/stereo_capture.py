#!/usr/bin/env python2.7

#  Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

""" Caputure stereo ZED camera images 
"""
import sys
import os
import cv2
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

    out_width      = int(stereo_width/2)
    out_height     = int(stereo_height)

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

    # create directories
    pathL = os.path.join(image_path, 'imageL')
    if os.path.exists(pathL) == 0:
        os.mkdir(pathL)

    pathR = os.path.join(image_path, 'imageR')
    if os.path.exists(pathR) == 0:
     os.mkdir(pathR)

    num = 1
    while vid_capture.isOpened():

        succes, img = vid_capture.read()
        k = cv2.waitKey(5)

        if k == ord('q'):
            break
        elif k == ord('s'): # wait for 's' key to save and exit
            imgL = img[ver_offset:out_height-ver_offset, 0:out_width]
            imgR = img[ver_offset:out_height-ver_offset, out_width:2*out_width]
            cv2.imwrite(pathL + '/image' + str(num).zfill(2) + '.png', imgL)
            cv2.imwrite(pathR + '/image' + str(num).zfill(2) + '.png', imgR)
            print("images saved!")
            num += 1

        cv2.imshow('Stereo Image',img)

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
    # valid camera_mode: 'FHD', 'FHD2',  HD', 'VGA'
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