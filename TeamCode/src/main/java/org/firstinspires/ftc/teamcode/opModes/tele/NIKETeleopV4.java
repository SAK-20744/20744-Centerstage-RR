package org.firstinspires.ftc.teamcode.opModes.tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.Arm1;
import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.Elbow;
import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.OutakeSingle;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.SAK26MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@Config
@Disabled
@TeleOp(name= "NIKE Teleop V4" , group = "advanced")
public class NIKETeleopV4 extends LinearOpMode {

    public static double SPEED = 0.8;
    public static double ELBOWSPEED = 1;
    //change x and y to match gamepad1.a positions
    private double arm1Position, arm2Position,wristPosition;
    private double armSpeed = 1;
    private boolean useWrist = true;
    private boolean backdrop = false;

    private double boardIMU = 0;
    private double pixelLevel = 1;

    private double height = 0;
    private AHRS navx_device;

    public static double pX = 0.045, iX = 0.02, dX = 0.05;
    public static double pY = 0.055, iY = 0, dY = 0.35;
    public static double pTurn = 0.045, iTurn = 0, dTurn = 0.05;


    @Override
    public void runOpMode() throws InterruptedException {


//        boolean targetFound = false;    // Set to true when an AprilTag target is detected
//        double aprilTagDrive = 0;        // Desired forward power/speed (-1 to +1)
//        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
//        double turn = 0;        // Desired turning power/speed (-1 to +1)
//
//        // Initialize the Apriltag Detection process
//        initAprilTag();
//
//        // Initialize the hardware variables. Note that the strings used here as parameters
//        // to 'get' must match the names assigned during the robot configuration.
//        // step (using the FTC Robot Controller app on the phone).
//        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
//        leftBackDrive  = hardwareMap.get(DcMotor.class, "bl");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
//
//        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
//        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
//        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
//
//        if (USE_WEBCAM)
//            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
//
//        // Wait for driver to press start
//        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
//        telemetry.addData(">", "Touch Play to start OpMode");

        DcMotor left_lift;
        DcMotor right_lift;
        DcMotor elbow;

        CRServo intake;
        Servo door;
        Servo wrist;
        Servo plane;

        Arm1 arm1;
        Elbow arm2;

        double looptime = 0;
        boolean ButtonXBlock = false;
        double wristservoposition = 0;
        boolean ButtonOBlock = false;

        boolean intaking = false;
        boolean hanging = false;

        navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"), AHRS.DeviceDataType.kProcessedData);

        SAK26MecanumDrive drive = new SAK26MecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(new Pose2d(0,0,0));

        PIDFController speedController;
        PIDFController strafeController;
        PIDFController turnController;

        speedController = new PIDFController(pX, iX, dX, 0);
        strafeController = new PIDFController(pY, iY, dY, 0);
        turnController = new PIDFController(pTurn, iTurn, dTurn, 0);

        left_lift = hardwareMap.get(DcMotor.class, "left_lift");
        right_lift = hardwareMap.get(DcMotor.class, "right_lift");
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        intake = hardwareMap.get(CRServo.class, "intake");
        door = hardwareMap.get(Servo.class, "door");
        wrist = hardwareMap.get(Servo.class, "wrist");
        plane = hardwareMap.get(Servo.class, "plane");

        arm1 = (new Arm1(hardwareMap));
        arm2 = new Elbow(hardwareMap);

        OutakeSingle outake = new OutakeSingle(hardwareMap);

//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(
//                new IMU.Parameters(
////                        new RevHubOrientationOnRobot(RevHubOrientationOnRobot.xyzOrientation(Math.toDegrees(0),Math.toDegrees(0),Math.toDegrees(-90)))
//                        new RevHubOrientationOnRobot(
//                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
//                        )
//                )
//        );

        waitForStart();

        if (isStopRequested()) return;

        useWrist = true;
        backdrop = false;
        arm1Position=178;
        arm2Position=-30;

        while (opModeIsActive() && !isStopRequested()) {
//
//
//            targetFound = false;
//            desiredTag = null;
//
//            // Step through the list of detected tags and look for a matching tag
//            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//            for (AprilTagDetection detection : currentDetections) {
//                // Look to see if we have size info on this tag.
//                if (detection.metadata != null) {
//                    //  Check to see if we want to track towards this tag.
//                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
//                        // Yes, we want to use this tag.
//                        targetFound = true;
//                        desiredTag = detection;
//                        break;  // don't look any further.
//                    } else {
//                        // This tag is in the library, but we do not want to track it right now.
//                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
//                    }
//                } else {
//                    // This tag is NOT in the library, so we don't have enough information to track to it.
//                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
//                }
//            }
//            // Tell the driver what we see, and what to do.
//            if (targetFound) {
//                telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
//                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
//                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
//                telemetry.addData("elevation", "%3.0f degrees", desiredTag.ftcPose.elevation);
//                telemetry.addData("pitch", "%3.0f degrees", desiredTag.ftcPose.pitch);
//            } else {
//                telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
//            }
//            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
//
//            if (gamepad1.left_bumper && targetFound) {
//
//                turn = turnController.calculate(targetTurn, desiredTag.ftcPose.pitch);
////                strafe = (strafeController.calculate(targetStrafe, desiredTag.ftcPose.elevation));
//                aprilTagDrive = speedController.calculate(targetDrive, desiredTag.ftcPose.range);
//                strafe = gamepad1.left_stick_x;
//
//                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", aprilTagDrive, strafe, turn);
//                moveRobot(aprilTagDrive, strafe, turn);
////                sleep(10);
//            }

            /* field centric

//            Pose2d poseEstimate = drive.getPoseEstimate();
//
//            Vector2d input = new Vector2d(
//                    -gamepad1.left_stick_y,
//                    -gamepad1.left_stick_x
//            ).rotated(-poseEstimate.getHeading());
//
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            input.getX(),
//                            input.getY(),
//                            -gamepad1.right_stick_x
//                    )
//            );
//
//            drive.update();

             */

//            ROBOT CENTRIC

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -(gamepad1.left_stick_y),
                            -(gamepad1.left_stick_x),
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            if (gamepad1.left_bumper)
                door.setPosition(0);
            else
                door.setPosition(0.95);

            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                plane.setPosition(0.6);
            } else {
                plane.setPosition(0.2);
            }

            if(gamepad1.right_bumper || gamepad2.right_bumper) {
                intake.setPower(-1);
                intaking = true;
            }
            else if(gamepad1.y) {
                intake.setPower(1);
            }
            else if(gamepad1.x) {
                intake.setPower(1);
                intaking = true;
            }
            else {
                intake.setPower(0);
                intaking = false;
            }

            if(gamepad2.dpad_down || gamepad1.dpad_down) {
                pixelLevel -= 1;
                sleep(200);
            }
            if(gamepad2.dpad_up || gamepad1.dpad_up) {
                pixelLevel += 1;
                sleep(200);
            }
            
            if(pixelLevel < 0)
                pixelLevel = 0;
            if(pixelLevel > 9)
                pixelLevel = 9;
            
            if(gamepad1.a){
                //Intaking
                useWrist = true;
                backdrop = false;
                arm1Position=178;
                arm2Position=-30;
                armSpeed = 0.7;
            }

            if(gamepad1.b) {
                backdrop = true;
                if(pixelLevel < 1)
                    pixelLevel = 1;
                if(pixelLevel > 9)
                    pixelLevel = 9;
            }

            if(gamepad2.y){
                //Pre-Hang
                backdrop = false;
                intaking = false;
                useWrist = false;
                arm1Position=83;
                arm2Position=254;
                wristPosition=0.0;
                armSpeed = 1;
            }
            if(gamepad2.a){
                //Hang
                backdrop = false;
                intaking = false;
                useWrist = false;
                arm1Position=115;
                arm2Position=152;
                wristPosition=0.8;
                armSpeed = 1;
            }

            if(backdrop){
                //Scoring
                intaking = false;
                useWrist = false;

                if(gamepad2.dpad_down) {
                    pixelLevel -= 1;
                    sleep(200);
                }
                if(gamepad2.dpad_up) {
                    pixelLevel += 1;
                    sleep(200);
                }

                if(pixelLevel < 1)
                    pixelLevel = 1;
                if(pixelLevel > 9)
                    pixelLevel = 9;

                if (gamepad1.left_bumper)
                    door.setPosition(0);
                else
                    door.setPosition(0.95);

                if(pixelLevel == 1){
                    arm1Position=120;
                    arm2Position=26;
                    wristPosition=1.00;
                    armSpeed = 0.6;
                }
                else if(pixelLevel == 2){
                    arm1Position=120;
                    arm2Position=36;
                    wristPosition=0.59;
                    armSpeed = 0.75;
                }
                else if(pixelLevel == 3){
                    arm1Position=120;
                    arm2Position=43;
                    wristPosition=0.49;
                    armSpeed = 0.85;
                }
                else if(pixelLevel == 4){
                    arm1Position=118;
                    arm2Position=51;
                    wristPosition=0.42;
                    armSpeed = 0.9;
                }
                else if(pixelLevel == 5){
                    arm1Position=117;
                    arm2Position=61;
                    wristPosition=0.34;
                    armSpeed = 1;
                }
                else if(pixelLevel == 6){
                    arm1Position=120;
                    arm2Position=69;
                    wristPosition=0.31;
                    armSpeed = 1;
                }
                else if(pixelLevel == 7){
                    arm1Position=109;
                    arm2Position=85;
                    wristPosition=0.31;
                    armSpeed = 1;
                }
                else if(pixelLevel == 8){
                    arm1Position=100;
                    arm2Position=98;
                    wristPosition=0.31;
                    armSpeed = 1;
                }
                else if(pixelLevel == 9){
                    arm1Position=95;
                    arm2Position=106;
                    wristPosition=0.31;
                    armSpeed = 1;
                }

            }

//            outake.IVKtoArmPoses(arm1Position, arm2Position, wristPosition, useWrist, intaking,armSpeed);

            telemetry.addData("Pixel Level:", pixelLevel);
            telemetry.addData("Arm1 Degrees:" , outake.getMotorArmDeg());
            telemetry.addData("Arm1 Pos:" , outake.getMotorArmPos());
            telemetry.addData("Arm2 Degrees:" , outake.getServoArmDeg());
            telemetry.addData("Arm2 Pos:" , outake.getServoArmPos());
            telemetry.addData("Wrist Degrees:" , outake.getWristDeg());
            telemetry.addData("Wrist Pos:" , outake.getWristPos());

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - looptime));
            looptime = loop;


            telemetry.update();
        }
    }


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
     * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target elevation to turn the robot.)
     * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target pitch to strafe the robot)
     * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
     *
     * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
     * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
     *
     * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
     * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
     *
     */

    // Adjust these numbers to suit your robot.
//    public static double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
//
//    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
//    //  applied to the drive motors to correct the error.
//    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
////    public static double SPEED_GAIN  =  0.2  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
//    public static double STRAFE_GAIN =  0.025 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree pitch error.   (0.25 / 25.0)
//    public static double TURN_GAIN   =  0.065  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
//
////    public static double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
//    public static double MAX_AUTO_STRAFE= 0.65;   //  Clip the approach speed to this max value (adjust for your robot)
//    public static double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    public static double targetTurn = 0;
    public static double targetStrafe = 0;
    public static double targetDrive = 15;

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
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
     * Positive pitch is counter-clockwise
     */
    public void moveRobot(double x, double y, double pitch) {
        // Calculate wheel powers.
        double leftFrontPower    =  x - y - pitch;
        double rightFrontPower   =  x + y + pitch;
        double leftBackPower     =  x + y - pitch;
        double rightBackPower    =  x - y + pitch;

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
                    .addProcessor(aprilTag)
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
}


