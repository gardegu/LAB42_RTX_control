#!/usr/bin/env python3

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

import os
import sys
import numpy as np
import cv2
import configparser
import argparse

def parse_calib_file(calib_file_path, camera_mode='HD', width=1280, height=720):
    """ parse ZED camera calibration file
    calib_file_path: calibration file path
    """
    camera_calibration = configparser.ConfigParser()
    camera_calibration.read(calib_file_path)
    # sections in calibration file
    left   = camera_calibration['LEFT_CAM_'+camera_mode]
    right  = camera_calibration['RIGHT_CAM_'+camera_mode]
    common = camera_calibration['STEREO']
    # LEFT_CAM] intrinsics
    l_fx = float(left['fx'])
    l_fy = float(left['fy'])
    l_cx = float(left['cx'])
    l_cy = float(left['cy'])
    l_k1 = float(left['k1'])
    l_k2 = float(left['k2'])
    l_k3 = float(left['k3'])
    l_p1 = float(left['p1'])
    l_p2 = float(left['p2'])
    #[RIGHT_CAM] intrinsics
    r_fx = float(right['fx'])
    r_fy = float(right['fy'])
    r_cx = float(right['cx'])
    r_cy = float(right['cy'])
    r_k1 = float(right['k1'])
    r_k2 = float(right['k2'])
    r_k3 = float(right['k3'])
    r_p1 = float(right['p1'])
    r_p2 = float(right['p2'])
    # [STEREO] extrinsics
    baseline = float(common['Baseline'])*1e-3 # in meter
    try:
        ty   = float(common['TY'])
    except:
        ty   = 0.0
    try:
        tz   = float(common['TZ'])
    except:
        tz   = 0.0
    ry   = float(common['CV_'+camera_mode])
    rx   = float(common['RX_'+camera_mode])
    rz   = float(common['RZ_'+camera_mode])
    # rotation matrix: right camera rotation matrix w.r.t. left camera
    rvec = np.array([rx, ry, rz]).reshape(-1,1) # col vector
    R    = cv2.Rodrigues(rvec)[0]
    # left camera params
    K1   = np.array([ [l_fx, 0.,   l_cx],
                      [0.,   l_fy, l_cy],
                      [0.,   0.,   1.  ] ])
    # right camera params
    K2   = np.array([ [r_fx,  0.,   r_cx],
                      [0.,    r_fy, r_cy],
                      [0.,    0.,   1.  ] ])
    # left camera distortion
    D1   = np.array( [l_k1, l_k2, l_p1, l_p2, l_k3]).reshape(1,5)
    # right camera distortion
    D2   = np.array( [r_k1, r_k2, r_p1, r_p2, r_k3]).reshape(1,5)
    # translation between left and right cameras
    T    = np.array([(-1. * l_fx * baseline), ty, tz]).reshape(-1,1)
    # image size
    image_size_dict = {'2K':(2208, 1242), 'FHD':(1920, 1080), 'HD':(1280, 720), 'VGA':(672, 376)}
    image_size = image_size_dict[camera_mode] # (width, height)
    return K1, D1, K2, D2, image_size, R, T

def parse_calib_file_wrapper(config_file_path, camera_mode):
    """ wrapper function for 'parse_calib_file' to support new modes 'HD2' and 'FHD2'
    'HD2' : 720p resolution obtained by center-cropping 1080p
    'FHD2': 1920x1024 resolution obtained by cropping evenly top and bottom of 1080p
    """
    if camera_mode=='HD2':
        K1, D1, K2, D2, image_size, R, T = parse_calib_file(config_file_path, 'FHD')
        # update camera matrices and image size, others carried over
        l = (1920-1280)/2 # = 320
        t = (1080-720)/2  # = 180
        K1[0,2] = K1[0,2] - l # l_cx
        K1[1,2] = K1[1,2] - t # l_cy
        K2[0,2] = K2[0,2] - l # r_cx
        K2[1,2] = K2[1,2] - t # r_cy
        image_size = (1280, 720)
    elif camera_mode=='FHD2':
        K1, D1, K2, D2, image_size, R, T = parse_calib_file(config_file_path, 'FHD')
        # update camera matrices and image size, others carried over
        t = (1080-1024)/2  # = 180
        K1[1,2] = K1[1,2] - t # l_cy
        K2[1,2] = K2[1,2] - t # r_cy
        image_size = (1920, 1024)
    else: # all the other legacy modes
        K1, D1, K2, D2, image_size, R, T = parse_calib_file(config_file_path, camera_mode)
    return K1, D1, K2, D2, image_size, R, T

def save_camera_info(image_size, camera_name, dist_model, K, D, R, P, file_path):
    """ save the camera info to given file_path
    """
    cv_file = cv2.FileStorage(file_path, cv2.FILE_STORAGE_WRITE)
    # work-around. write() has a bug in writing integers (adding .)
    cv_file.write("image_width",             str(image_size[0]))
    cv_file.write("image_height",            str(image_size[1]))
    cv_file.write("camera_name",             camera_name)
    cv_file.write("camera_matrix",           K)
    cv_file.write("distortion_model",        dist_model)
    cv_file.write("distortion_coefficients", D)
    cv_file.write("rectification_matrix",    R)
    cv_file.write("projection_matrix",       P)
    cv_file.release()
    print("camera_info saved into {}".format(file_path))

def save_remap_lut(map_x, map_y, file_path):
    """ save remap LUT to given file_path
    """
    col0 = np.floor((map_x.flatten().reshape(-1,1)*16.)).astype(int)
    col1 = np.floor((map_y.flatten().reshape(-1,1)*16.)).astype(int)
    lut  = np.concatenate((col0, col1), axis=1)
    with open(file_path, "w") as f:
        np.savetxt(f, lut, fmt='%i')
        print("remap LUT saved into {}".format(file_path))

def save_LDC_lut(map_x, map_y, file_path):
    """ save LDC LUT to given file_path
    """
    LDC_DS_FACTOR = 4

    width   = map_x.shape[1]
    height  = map_x.shape[0]
    sWidth  = int(width / LDC_DS_FACTOR + 1)
    sHeight = int(height / LDC_DS_FACTOR + 1)
    lineOffset = ((sWidth + 15) & (~15))

    ldcLUT_x  = np.zeros((sHeight,lineOffset), dtype=np.int16)
    ldcLUT_y  = np.zeros((sHeight,lineOffset), dtype=np.int16)

    for y in range(0, sHeight):
        j = y * LDC_DS_FACTOR
        if j > height - 1:
            j = height - 1

        for x in range(0, sWidth):
            i = x * LDC_DS_FACTOR
            if i > width - 1:
                i = width - 1

            dx = np.floor(map_x[j, i]*8. + 0.5).astype(int) - i*8
            dy = np.floor(map_y[j, i]*8. + 0.5).astype(int) - j*8

            ldcLUT_x[y, x] = dx & 0xFFFF
            ldcLUT_y[y, x] = dy & 0xFFFF

        remain = ((sWidth + 15) & (~15)) - sWidth
        while (remain > 0):
            ldcLUT_x[y, sWidth - 1 + remain] = 0
            ldcLUT_y[y, sWidth - 1 + remain] = 0
            remain = remain - 1

    #  y offset comes first
    col0   = np.floor(ldcLUT_y.flatten().reshape(-1,1)).astype(np.int16)
    col1   = np.floor(ldcLUT_x.flatten().reshape(-1,1)).astype(np.int16)
    ldcLUT = np.concatenate((col0, col1), axis=1)
    ldcLUT.tofile(file_path)
    print("LDC LUT saved into {}".format(file_path))

def main(calib_file, camera_mode, driver_path):
    """ Parse ZED camera calibration files, generate camera_info for left and right image topics,
    and generate undistortion and rectification look-up-table (aks remap LUT).
    calib_file: ZED camera calibration file from factory. Should be stored under <zed_capture>/config folder
    {left,right} camera_info: <zed_capture>/config/<calib_file_SN_str>_camera_info_{left,right}.yaml
    {left,right} undist/rect LUT: <zed_capture>/config/<calib_file_SN_str>_remap_LUT_{left,right}.txt
    """
  	# calibration file path
    config_file_path = os.path.join(driver_path, "config", calib_file)
    if not os.path.exists(config_file_path):
        sys.exit(f'{config_file_path} does not exist.')

    # parse calibration file
    K1, D1, K2, D2, image_size, R, T = parse_calib_file_wrapper(config_file_path, camera_mode)
    # cv2.stereoRectify
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(cameraMatrix1=K1, distCoeffs1=D1,
                                                      cameraMatrix2=K2, distCoeffs2=D2,
                                                      imageSize=image_size, R=R, T=T,
                                                      flags=cv2.CALIB_ZERO_DISPARITY, alpha=0.0)
    #  create remap table for left
    map_left_x, map_left_y   = cv2.initUndistortRectifyMap(K1, D1, R1, P1, image_size, cv2.CV_32FC1)
    # create remap table for right
    map_right_x, map_right_y = cv2.initUndistortRectifyMap(K2, D2, R2, P2, image_size, cv2.CV_32FC1)
    # write left and right camera info
    sn_str = os.path.splitext(calib_file)[0]
    dist_model = "plumb_bob"
    file_path  = os.path.join(driver_path, "config", sn_str+"_"+camera_mode+"_camera_info_left.yaml")
    save_camera_info(image_size, "camera/left",  dist_model, K1, D1, R1, P1, file_path)
    file_path  = os.path.join(driver_path, "config", sn_str+"_"+camera_mode+"_camera_info_right.yaml")
    save_camera_info(image_size, "camera/right", dist_model, K2, D2, R2, P2, file_path)

    # # write undist/rect LUT
    # file_path  = os.path.join(driver_path, "config", sn_str+"_"+camera_mode+"_remap_LUT_left.txt")
    # save_remap_lut(map_left_x, map_left_y, file_path)
    # file_path  = os.path.join(driver_path, "config", sn_str+"_"+camera_mode+"_remap_LUT_right.txt")
    # save_remap_lut(map_right_x, map_right_y, file_path)

    # write LDC undist/rect LUT
    file_path  = os.path.join(driver_path, "config", sn_str+"_"+camera_mode+"_LUT_left.bin")
    save_LDC_lut(map_left_x, map_left_y, file_path)
    file_path  = os.path.join(driver_path, "config", sn_str+"_"+camera_mode+"_LUT_right.bin")
    save_LDC_lut(map_right_x, map_right_y, file_path)


if __name__ == "__main__":
    # input argument parser
    parser = argparse.ArgumentParser(description='ZED camera camera_info & LCD LUT generation tool')
    parser.add_argument('--input', '-i', type=str, default='SN5867575.conf',
        help='Factory calibration data filename for ZED camera. The calibration data file should be placed under <zed_capture>/config.')
    parser.add_argument('--mode', '-m', type=str, default=None,
        help="Camera mode. Valid camera mode: 2K, FHD, FHD2, HD, HD2, VGA")
    parser.add_argument('--path', '-p', type=str, default='../../ros1/drivers/zed_capture',
        help="Path to zed_capture driver node")
    args = parser.parse_args()

    # calibration file that is stored under <zed_capture>/config folder
    # TI-internal: 'SN5867575.conf':  ZED1 camera
    #              'SN29788442.conf': ZED2 camera
    calib_file = args.input

    # valid camera_mode: '2K', 'FHD', 'FHD2', HD', 'HD2', 'VGA'
    # 'HD2' : 720p with longer focal length by center-cropping 1080p.
    # 'FHD2': 1920x1024 by cropping evenly top and bottom of 1080p.
    if args.mode is None:
        camera_modes = ['HD', 'HD2', 'FHD', 'FHD2']
    else:
        camera_modes = [args.mode]

    driver_path = args.path

    # main routine
    for camera_mode in camera_modes:
        main(calib_file, camera_mode, driver_path)
