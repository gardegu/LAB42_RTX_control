# Stereo calibration
import os
import sys
import numpy as np
import cv2
import configparser
import argparse
import glob

def setStereoCameraMode(camera_mode):
    if camera_mode == 'VGA':
        stereo_width  = 1344
        stereo_height = 376
    elif camera_mode == 'HD':
        stereo_width  = 2560
        stereo_height = 720
    elif camera_mode == 'FHD':
        stereo_width  = 3840
        stereo_height = 1080
    elif camera_mode == 'FHD2':
        stereo_width  = 3840
        stereo_height = 1024
    else:
        sys.exit("Error: Wrong camer mode {}".format(camera_mode))

    out_width     = int(stereo_width/2)
    out_height    = int(stereo_height)

    return stereo_width, stereo_height, out_width, out_height


def findControlPoints(image_path, chessBoardSize, imageSize, gridSizeInMM):
    """ Find corner points from chess board
    """
    # termination criteria for subpel search
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objPt        = np.zeros((chessBoardSize[0] * chessBoardSize[1], 3), np.float32)
    objPt[:, :2] = np.mgrid[0:chessBoardSize[0], 0:chessBoardSize[1]].T.reshape(-1,2)

    # gridSizeInMM : chessboard grid size in mm
    objPt        = objPt * gridSizeInMM

    # 3D point in real world space
    objPoints  = []
    # 2D points in left image plane
    imgPointsL = []
    # 2D points in right image plane
    imgPointsR = []

    imagesLeft  = sorted(glob.glob(os.path.join(image_path, 'imageL/*.png')))
    assert imagesLeft != [], 'Found no images'

    imagesRight = sorted(glob.glob(os.path.join(image_path, 'imageR/*.png')))
    assert imagesRight != [], 'Found no images'

    for imgLeft, imgRight in zip(imagesLeft, imagesRight):
        # Show image number
        print(imgRight)

        imgL  = cv2.imread(imgLeft)
        imgR  = cv2.imread(imgRight)
        grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        retL, cornersL = cv2.findChessboardCorners(grayL, chessBoardSize, None)
        retR, cornersR = cv2.findChessboardCorners(grayR, chessBoardSize, None)

        # If found, add object points, image points
        if retL == True and retR == True:

            #print(cornersR)
            lastRow = chessBoardSize[0]*(chessBoardSize[1]-1)
            # Check if top and bottom rows are reversed
            # Compare y pos
            if cornersL[0, 0, 1] > cornersL[lastRow, 0, 1]:
                print("Points detected are NOT correct in Left image (Upside down). Excluded from calibration.")
                continue

            if cornersR[0, 0, 1] > cornersR[lastRow, 0, 1]:
                print("Points detected are NOT correct in Right image (Upside down). Excluded from calibration.")
                continue

            # Check if corner points' direction are reversed, i.e. rigth->left
            # Compare x pos
            if cornersL[0, 0, 0] > cornersL[1, 0, 0]:
                print("Points detected are NOT correct in Left image (Reverse direction). Excluded from calibration.")
                continue

            if cornersR[0, 0, 0] > cornersR[1, 0, 0]:
                print("Points detected are NOT correct in Right image (Reverse direction). Excluded from calibration.")
                continue

            objPoints.append(objPt)

            cornersL = cv2.cornerSubPix(grayL, cornersL, (11,11), (-1,-1), criteria)
            imgPointsL.append(cornersL)

            cornersR = cv2.cornerSubPix(grayR, cornersR, (11,11), (-1,-1), criteria)
            imgPointsR.append(cornersR)

            # Draw and display the corners
            cv2.drawChessboardCorners(imgL, chessBoardSize, cornersL, retL)
            cv2.imshow('Left Image', imgL)
            cv2.drawChessboardCorners(imgR, chessBoardSize, cornersR, retR)
            cv2.imshow('Right Image', imgR)
            cv2.waitKey(1000)

    cv2.destroyAllWindows()

    return objPoints, imgPointsL, imgPointsR, grayL, grayR


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


def createRemapTable(K1, D1, R1, P1, K2, D2, R2, P2, imageSize, out_folder, camera_mode):

    #  create remap table for left
    map_left_x, map_left_y   = cv2.initUndistortRectifyMap(K1, D1, R1, P1, imageSize, cv2.CV_32FC1)
    # create remap table for right
    map_right_x, map_right_y = cv2.initUndistortRectifyMap(K2, D2, R2, P2, imageSize, cv2.CV_32FC1)

    # write LDC undist/rect LUT
    file_path  = os.path.join(out_folder, "custom"+"_"+camera_mode+"_LUT_left.bin")
    save_LDC_lut(map_left_x, map_left_y, file_path)
    file_path  = os.path.join(out_folder, "custom"+"_"+camera_mode+"_LUT_right.bin")
    save_LDC_lut(map_right_x, map_right_y, file_path)


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


def main(camera_mode, image_path, chess_width, chess_height, chess_length, out_folder):
    """Main function"""

    stereo_width, stereo_height, out_width, out_height = \
        setStereoCameraMode(camera_mode)

    # Find the chessboard corners
    chessBoardSize = (chess_width, chess_height)
    imageSize      = (out_width, out_height)

    objPoints, imgPointsL, imgPointsR, grayL, grayR = \
        findControlPoints(image_path, chessBoardSize, imageSize, chess_length)


    # Calibrate left camera
    ret1, K1, D1, R1, T1 = cv2.calibrateCamera(objPoints, imgPointsL, grayL.shape[::-1], None, None)

    # Calibrate right camera
    ret2, K2, D2, R2, T2 = cv2.calibrateCamera(objPoints, imgPointsR, grayR.shape[::-1], None, None)

    # Calibrate stereo pair
    flags = 0
    #flags |= cv2.CALIB_FIX_INTRINSIC
    flags |= cv2.CALIB_USE_INTRINSIC_GUESS
    criteria_stereo= (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-6)

    retStereo, K1, D1, K2, D2, R, T, essentialMatrix, fundamentalMatrix = \
        cv2.stereoCalibrate(objPoints, imgPointsL, imgPointsR, K1, D1, K2, D2, imageSize,
                            criteria=criteria_stereo, flags=flags)

    # Print calibration error
    print("============================")
    print("Calibration RMS error")
    print("Left   = " + str(ret1))
    print("Right  = " + str(ret2))
    print("Stereo = " + str(retStereo))
    print("============================")

    # Do stereo rectification
    R1, R2, P1, P2, Q, roi1, roi2 = \
        cv2.stereoRectify(cameraMatrix1=K1, distCoeffs1=D1,
                          cameraMatrix2=K2, distCoeffs2=D2,
                          imageSize=imageSize, R=R, T=T,
                          flags=cv2.CALIB_ZERO_DISPARITY, alpha=0.0)

    # Create LDC remap tables
    createRemapTable(K1, D1, R1, P1, K2, D2, R2, P2, imageSize, out_folder, camera_mode)

    # Save camera info for ROS apps
    dist_model = "plumb_bob"
    file_path  = os.path.join(out_folder, "custom"+"_"+camera_mode+"_camera_info_left.yaml")
    save_camera_info(imageSize, "camera/left",  dist_model, K1, D1, R1, P1, file_path)
    file_path  = os.path.join(out_folder, "custom"+"_"+camera_mode+"_camera_info_right.yaml")
    save_camera_info(imageSize, "camera/right", dist_model, K2, D2, R2, P2, file_path)

    # Save Remap tables for test: Used by stereo_test.py
    stereoMapL = cv2.initUndistortRectifyMap(K1, D1, R1, P1, grayL.shape[::-1], cv2.CV_16SC2)
    stereoMapR = cv2.initUndistortRectifyMap(K2, D2, R2, P2, grayR.shape[::-1], cv2.CV_16SC2)

    print("Save Remap table (stereoMap.xml)...!")
    cv_file = cv2.FileStorage(os.path.join(image_path, 'stereoMap.xml'), cv2.FILE_STORAGE_WRITE)

    cv_file.write('stereoMapL_x',stereoMapL[0])
    cv_file.write('stereoMapL_y',stereoMapL[1])
    cv_file.write('stereoMapR_x',stereoMapR[0])
    cv_file.write('stereoMapR_y',stereoMapR[1])

    cv_file.release()


if __name__ == "__main__":
    # input argument parser
    parser = argparse.ArgumentParser(description='Stereo ZED camera capture')

    # Path to directory
    parser.add_argument('-p', '--path', type=str, default='./stereo_calib',
        help="Path to the folder for storing images for calibration")
    # Output folder for saving camera_info_*.yaml and LDC remap tables
    parser.add_argument('-o', '--outfolder', type=str, default='../../ros1/drivers/zed_capture/config',
        help="Path to the output folder for storing images for calibration")
    # valid camera_mode: 'FHD', 'FHD2', HD', 'VGA'
    # FHD2: 1920x1024
    parser.add_argument('-m', '--mode',type=str, default='HD',
        help="Camera mode. Valid camera mode: FHD, FHD2, HD, VGA")
    # Chess board width in terms of the number of control points
    parser.add_argument('-c', '--column', type=int, default=8,
        help="Chessboard width")
    # Chess board height in terms of the number of control points
    parser.add_argument('-r', '--row', type=int, default=6,
        help="Chessboard height")
    # Grid size in mm
    parser.add_argument('-s', '--size', type=int, default=25,
        help="Chessboard grid size in mm")

    args = parser.parse_args()

    camera_mode  = args.mode
    image_path   = args.path
    out_folder   = args.outfolder
    print(out_folder)
    chess_width  = args.column
    chess_height = args.row
    chess_length = args.size

    main(camera_mode, image_path, chess_width, chess_height, chess_length, out_folder)