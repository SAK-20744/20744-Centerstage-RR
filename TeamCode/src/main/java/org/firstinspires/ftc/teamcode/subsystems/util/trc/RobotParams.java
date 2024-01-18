/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.subsystems.util.trc;

import org.openftc.easyopencv.OpenCvCameraRotation;

//import TrcCommonLib.trclib.TrcDriveBase.DriveOrientation;
//import TrcCommonLib.trclib.TrcHomographyMapper;
//import TrcCommonLib.trclib.TrcPidController;
//import TrcCommonLib.trclib.TrcPose2D;
//import TrcCommonLib.trclib.TrcUtil;
//import TrcFtcLib.ftclib.FtcGamepad;

/**
 * This class contains robot and subsystem constants and parameters.
 */
public class RobotParams {

    public static final double FULL_FIELD_INCHES = 141.24;
    public static final double HALF_FIELD_INCHES = FULL_FIELD_INCHES / 2.0;
    public static final double FULL_TILE_INCHES = FULL_FIELD_INCHES / 6.0;
    public static final double ROBOT_LENGTH = 18.00;
    public static final int[] BLUE_BACKDROP_APRILTAGS = new int[]{1, 2, 3};
    public static final int[] RED_BACKDROP_APRILTAGS = new int[]{4, 5, 6};
    // AprilTag locations to place the pixel in inches.
    // DO NOT CHANGE the AprilTag location numbers. They are from the AprilTag metadata.
    public static final double APRILTAG_BACKDROP_X = 60.25;
    public static final double APRILTAG_AUDIENCE_WALL_X = -70.25;
    public static final double BACKDROP_APRILTAG_DELTA_Y = 6.0;
    // All AprilTags are at the height of 4.0-inch except for AprilTag 7 and 10 which are at the height of 5.5-inch.
    public static final TrcPose2D[] APRILTAG_POSES = new TrcPose2D[]{
            new TrcPose2D(APRILTAG_BACKDROP_X, 41.41, 90.0),        // TagId 1
            new TrcPose2D(APRILTAG_BACKDROP_X, 35.41, 90.0),        // TagId 2
            new TrcPose2D(APRILTAG_BACKDROP_X, 29.41, 90.0),        // TagId 3
            new TrcPose2D(APRILTAG_BACKDROP_X, -29.41, 90.0),       // TagId 4
            new TrcPose2D(APRILTAG_BACKDROP_X, -35.41, 90.0),       // TagId 5
            new TrcPose2D(APRILTAG_BACKDROP_X, -41.41, 90.0),       // TagId 6
            new TrcPose2D(APRILTAG_AUDIENCE_WALL_X, -40.63, -90.0), // TagId 7
            new TrcPose2D(APRILTAG_AUDIENCE_WALL_X, -35.13, -90.0), // TagId 8
            new TrcPose2D(APRILTAG_AUDIENCE_WALL_X, 35.13, -90.0),  // TagId 9
            new TrcPose2D(APRILTAG_AUDIENCE_WALL_X, 40.63, -90.0)   // TagId 10
    };
    //
    // Vision subsystem.
    //
    public static final int CAM_IMAGE_WIDTH = 640;
    public static final int CAM_IMAGE_HEIGHT = 480;
    public static final OpenCvCameraRotation CAM_ORIENTATION = OpenCvCameraRotation.UPRIGHT;
    // Camera location on robot.
    public static final double FRONTCAM_X_OFFSET = 0.0;
    public static final double FRONTCAM_Y_OFFSET = -(ROBOT_LENGTH / 2.0 - 2.5);
    public static final double FRONTCAM_Z_OFFSET = 9.75;
    public static final TrcPose2D FRONTCAM_POSE = new TrcPose2D(
            FRONTCAM_X_OFFSET, FRONTCAM_Y_OFFSET, 0.0);
    // TODO: Need to measure the back camera position offsets.
    public static final double BACKCAM_X_OFFSET = 0.0;
    public static final double BACKCAM_Y_OFFSET = (ROBOT_LENGTH / 2.0 - 2.125);
    public static final double BACKCAM_Z_OFFSET = 6.25;
    public static final TrcPose2D BACKCAM_POSE = new TrcPose2D(
            BACKCAM_X_OFFSET, BACKCAM_Y_OFFSET, 180.0);
//    // Camera: Micorosoft Lifecam HD 3000 v1/v2
//    public static final double WEBCAM_FX                        = 678.154;  // in pixels
//    public static final double WEBCAM_FY                        = 678.170;  // in pixels
//    public static final double WEBCAM_CX                        = 318.135;  // in pixels
//    public static final double WEBCAM_CY                        = 228.374;  // in pixels
//    // Camera: Logitech C270
//    public static final double WEBCAM_FX                        = 822.317;  // in pixels
//    public static final double WEBCAM_FY                        = 822.317;  // in pixels
//    public static final double WEBCAM_CX                        = 319.495;  // in pixels
//    public static final double WEBCAM_CY                        = 242.502;  // in pixels
//    // Camera: Logitech C310
//    public static final double WEBCAM_FX                        = 821.993;  // in pixels
//    public static final double WEBCAM_FY                        = 821.993;  // in pixels
//    public static final double WEBCAM_CX                        = 330.489;  // in pixels
//    public static final double WEBCAM_CY                        = 248.997;  // in pixels
//    // Camera: Logitech C920
//    public static final double WEBCAM_FX                        = 622.001;  // in pixels
//    public static final double WEBCAM_FY                        = 622.001;  // in pixels
//    public static final double WEBCAM_CX                        = 319.803;  // in pixels
//    public static final double WEBCAM_CY                        = 241.251;  // in pixels

    // Measurement unit: pixels
    // TODO: Tune these!
    public static final double HOMOGRAPHY_CAMERA_TOPLEFT_X = 0.0;
    public static final double HOMOGRAPHY_CAMERA_TOPLEFT_Y = 120.0;
    public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_X = CAM_IMAGE_WIDTH - 1;
    public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_Y = 120.0;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_X = 0.0;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y = CAM_IMAGE_HEIGHT - 1;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X = CAM_IMAGE_WIDTH - 1;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y = CAM_IMAGE_HEIGHT - 1;
    // Measurement unit: inches
    public static final double HOMOGRAPHY_WORLD_TOPLEFT_X = -12.5625;
    public static final double HOMOGRAPHY_WORLD_TOPLEFT_Y = 48.0 - ROBOT_LENGTH / 2.0 - FRONTCAM_Y_OFFSET;
    public static final double HOMOGRAPHY_WORLD_TOPRIGHT_X = 11.4375;
    public static final double HOMOGRAPHY_WORLD_TOPRIGHT_Y = 44.75 - ROBOT_LENGTH / 2.0 - FRONTCAM_Y_OFFSET;
    public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_X = -2.5625;
    public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_Y = 21.0 - ROBOT_LENGTH / 2.0 - FRONTCAM_Y_OFFSET;
    public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_X = 2.5626;
    public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y = 21.0 - ROBOT_LENGTH / 2.0 - FRONTCAM_Y_OFFSET;

    public static final TrcHomographyMapper.Rectangle cameraRect = new TrcHomographyMapper.Rectangle(
            RobotParams.HOMOGRAPHY_CAMERA_TOPLEFT_X, RobotParams.HOMOGRAPHY_CAMERA_TOPLEFT_Y,
            RobotParams.HOMOGRAPHY_CAMERA_TOPRIGHT_X, RobotParams.HOMOGRAPHY_CAMERA_TOPRIGHT_Y,
            RobotParams.HOMOGRAPHY_CAMERA_BOTTOMLEFT_X, RobotParams.HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y,
            RobotParams.HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X, RobotParams.HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y);
    public static final TrcHomographyMapper.Rectangle worldRect = new TrcHomographyMapper.Rectangle(
            RobotParams.HOMOGRAPHY_WORLD_TOPLEFT_X, RobotParams.HOMOGRAPHY_WORLD_TOPLEFT_Y,
            RobotParams.HOMOGRAPHY_WORLD_TOPRIGHT_X, RobotParams.HOMOGRAPHY_WORLD_TOPRIGHT_Y,
            RobotParams.HOMOGRAPHY_WORLD_BOTTOMLEFT_X, RobotParams.HOMOGRAPHY_WORLD_BOTTOMLEFT_Y,
            RobotParams.HOMOGRAPHY_WORLD_BOTTOMRIGHT_X, RobotParams.HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y);
}