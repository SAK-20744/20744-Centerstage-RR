package org.firstinspires.ftc.teamcode.auto;
import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
//import org.firstinspires.ftc.teamcode.common.vision.PropPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm1;
import org.firstinspires.ftc.teamcode.subsystems.ServoArm;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous
@Disabled
public class AutoStack extends LinearOpMode {


        /*
         * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
         * The code assumes a Holonomic (Mecanum or X Drive) Robot.
         *
         * For an introduction to AprilTags, see the ftc-docs link below:
         * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
         *
         * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
         * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
         * https://ftc-docs.firstinspires.org/apriltag-detection-values
         *
         * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
         * driving towards the tag to achieve the desired distance.
         * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
         * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
         *
         * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
         * The motor directions must be set so a positive power goes forward on all wheels.
         * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
         * so you should choose to approach a valid tag ID (usually starting at 0)
         *
         * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
         * Manually drive the robot until it displays Target data on the Driver Station.
         *
         * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
         * Release the Left Bumper to return to manual driving mode.
         *
         * Under "Drive To Target" mode, the robot has three goals:
         * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
         * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
         * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
         *
         * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
         * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
         *
         * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
         * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
         *
         */

        public static double pX = 0.045, iX = 0, dX = 0.05;

        // Adjust these numbers to suit your robot.
        public static double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

        //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
        //  applied to the drive motors to correct the error.
        //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    //    public static double SPEED_GAIN  =  0.2  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        public static double STRAFE_GAIN =  0.025 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        public static double TURN_GAIN   =  0.065  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        //    public static double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        public static double MAX_AUTO_STRAFE= 0.65;   //  Clip the approach speed to this max value (adjust for your robot)
        public static double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

        private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
        private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
        private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
        private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

        private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
        private static final int DESIRED_TAG_ID = 9;     // Choose the tag you want to approach or set to -1 for ANY tag.
//        private PropPipeline propPipeline;
        private VisionPortal visionPortal;               // Used to manage the video source.
        private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
        private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

        /**
         * Move robot according to desired axes motions
         * <p>
         * Positive X is forward
         * <p>
         * Positive Y is strafe left
         * <p>
         * Positive Yaw is counter-clockwise
         */
        public void moveRobot(double x, double y, double yaw) {
            // Calculate wheel powers.
            double leftFrontPower    =  x -y -yaw;
            double rightFrontPower   =  x +y +yaw;
            double leftBackPower     =  x +y -yaw;
            double rightBackPower    =  x -y +yaw;

            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send powers to the wheels.
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
        }

        /**
         * Initialize the AprilTag processor.
         */
        private void initAprilTag() {
            // Create the AprilTag processor by using a builder.
            aprilTag = new AprilTagProcessor.Builder().build();

            // Adjust Image Decimation to trade-off detection-range for detection-rate.
            // eg: Some typical detection data using a Logitech C920 WebCam
            // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
            // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
            // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
            // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
            // Note: Decimation can be changed on-the-fly to adapt during a match.
            aprilTag.setDecimation(2);

            // Create the vision portal by using a builder.
            if (USE_WEBCAM) {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                        .setCameraResolution(new Size(1920,1080))
                        .addProcessor(aprilTag)
//                        .addProcessor(propPipeline)
                        .enableLiveView(true)
                        .setAutoStopLiveView(true)
                        .build();
            } else {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(BuiltinCameraDirection.BACK)
                        .addProcessor(aprilTag)
                        .build();
            }
        }

        /*
         Manually set the camera gain and exposure.
         This can only be called AFTER calling initAprilTag(), and only works for Webcams;
        */
        private void    setManualExposure(int exposureMS, int gain) {
            // Wait for the camera to be open, then use the controls

            if (visionPortal == null) {
                return;
            }

            // Make sure camera is streaming before we try to set the exposure controls
            if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                telemetry.addData("Camera", "Waiting");
                telemetry.update();
                while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                    sleep(20);
                }
                telemetry.addData("Camera", "Ready");
                telemetry.update();
            }

            // Set camera controls unless we are stopping.
            if (!isStopRequested())
            {
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    sleep(50);
                }
                exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
                sleep(20);
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                gainControl.setGain(gain);
                sleep(20);
            }
        }




        @Override
        public void runOpMode() {

            boolean targetFound = false;    // Set to true when an AprilTag target is detected
            double aprilTagDrive = 0;        // Desired forward power/speed (-1 to +1)
            double strafe = 0;        // Desired strafe power/speed (-1 to +1)
            double turn = 0;        // Desired turning power/speed (-1 to +1)

            PIDFController speedController;
    //        PIDFController strafeController;
    //        PIDFController turnController;

            speedController = new PIDFController(pX, iX, dX, 0);
    //        strafeController = new PIDFController(pY, iY, dY, 0);
    //        turnController = new PIDFController(pTurn, iTurn, dTurn, 0);

            // Initialize the Apriltag Detection process
            initAprilTag();

            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must match the names assigned during the robot configuration.
            // step (using the FTC Robot Controller app on the phone).
            leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
            leftBackDrive  = hardwareMap.get(DcMotor.class, "bl");
            rightBackDrive = hardwareMap.get(DcMotor.class, "br");

            // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
            // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
            // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            if (USE_WEBCAM)
                setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

            // Wait for driver to press start
            telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");


            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            Arm1 arm1 = (new Arm1(hardwareMap));
            ServoArm arm2 = new ServoArm(hardwareMap);

            Trajectory tin = drive.trajectoryBuilder(new Pose2d())
                    .forward(20)
                    .build();

            Servo wrist = hardwareMap.get(Servo.class, "wrist");
            DcMotor left_lift = hardwareMap.get(DcMotor.class, "left_lift");
            DcMotor right_lift = hardwareMap.get(DcMotor.class, "right_lift");
            CRServo leftArm = hardwareMap.get(CRServo.class, "leftArm");
            CRServo rightArm = hardwareMap.get(CRServo.class, "rightArm");
            CRServo intake = hardwareMap.get(CRServo.class, "intake");
            Servo door = hardwareMap.get(Servo.class, "door");

            boolean ButtonXBlock = false;
            double wristservoposition = 0.63;

            // move motors to -660 for init
            wrist.setPosition(wristservoposition);

            while (!(opModeIsActive() || isStopRequested())) {
                left_lift.setPower(-gamepad2.right_stick_y);
                right_lift.setPower(-gamepad2.right_stick_y);
                leftArm.setPower(-gamepad2.left_stick_y);
                rightArm.setPower(gamepad2.left_stick_y);
                telemetry.addData("Left Lift Encoder", left_lift.getCurrentPosition());
                telemetry.addData("Right Lift Encoder", right_lift.getCurrentPosition());
                telemetry.addData("Servo Arm", arm2.getLocation());
                telemetry.update();
                if (gamepad2.a) {
                    left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    left_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }


            waitForStart();

            if (!isStopRequested()) {



                wrist.setPosition(0.16);
                sleep(500);
                drive.followTrajectory(tin);
                sleep(500);

                desiredTag = null;

                // Step through the list of detected tags and look for a matching tag
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    // Look to see if we have size info on this tag.
                    if (detection.metadata != null) {
                        //  Check to see if we want to track towards this tag.
                        if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                            // Yes, we want to use this tag.
                            targetFound = true;
                            desiredTag = detection;
                            break;  // don't look any further.
                        } else {
                            // This tag is in the library, but we do not want to track it right now.
                            telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                        }
                    } else {
                        // This tag is NOT in the library, so we don't have enough information to track to it.
                        telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                    }
                }

                    // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                    double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                    double headingError = desiredTag.ftcPose.bearing;
                    double yawError = desiredTag.ftcPose.yaw;

                    // Use the speed and turn "gains" to calculate how we want the robot to move.
    //                aprilTagDrive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                    aprilTagDrive = speedController.calculate(DESIRED_DISTANCE, desiredTag.ftcPose.range);
    //                turn = turnController.calculate()

                    telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", aprilTagDrive, strafe, turn);

                while(rangeError > 0.5){

                    currentDetections = aprilTag.getDetections();
                    for (AprilTagDetection detection : currentDetections) {
                        // Look to see if we have size info on this tag.
                        if (detection.metadata != null) {
                            //  Check to see if we want to track towards this tag.
                            if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                                // Yes, we want to use this tag.
                                targetFound = true;
                                desiredTag = detection;
                                break;  // don't look any further.
                            } else {
                                // This tag is in the library, but we do not want to track it right now.
                                telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                            }
                        } else {
                            // This tag is NOT in the library, so we don't have enough information to track to it.
                            telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                        }
                    }

                    moveRobot(aprilTagDrive, strafe, turn);
                    rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                    sleep(10);
                }

                intake.setPower(-0.3);
                sleep(500);
                intake.setPower(0);
                sleep(700);
                wrist.setPosition(0.14);
                sleep(500);
                intake.setPower(-0.3);
                sleep(500);
                intake.setPower(0);

                sleep(1000);
                leftArm.setPower(0.45);
                rightArm.setPower(-0.45);
                sleep(300);
                leftArm.setPower(0);
                rightArm.setPower(0);
                door.setPosition(0);
                sleep(2000);
                door.setPosition(0.95);
                sleep(500);

    //start


        }
    }
}