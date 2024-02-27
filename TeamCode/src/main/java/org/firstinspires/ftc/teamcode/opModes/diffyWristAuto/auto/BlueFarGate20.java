//package org.firstinspires.ftc.teamcode.opModes.diffyWristAuto.auto;
//
//import static org.firstinspires.ftc.teamcode.subsystems.vision.old.PropPipeline.Location.CENTER;
//import static org.firstinspires.ftc.teamcode.subsystems.vision.old.PropPipeline.Location.LEFT;
//import static org.firstinspires.ftc.teamcode.subsystems.vision.old.PropPipeline.Location.RIGHT;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
//import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.Arm1;
//import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.Elbow;
//import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.ServoDiffyWrist;
//import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.opmode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.subsystems.vision.old.PropPipeline;
//import org.firstinspires.ftc.teamcode.subsystems.vision.old.PropPipeline.Location;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.List;
//import java.util.concurrent.TimeUnit;
//
////@Disabled
//@Autonomous(name = "Blue Far Gate 2+0")
//public class BlueFarGate20 extends LinearOpMode {
//
//    private PropPipeline propPipeline;
//    private VisionPortal portal;
//    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
//    private AprilTagDetection desiredTag = null;
//
//    private DcMotor leftFrontDrive   = null;
//    private DcMotor rightFrontDrive  = null;
//    private DcMotor leftBackDrive    = null;
//    private DcMotor rightBackDrive   = null;
//
//    private static final int DESIRED_TAG_ID = 1; // LEFT April Tag - Aligns the robot to the Center
//
//    private double pX = 0.045, iX = 0.02, dX = 0.05;
//    private double pY = 0.055, iY = 0, dY = 0.35;
//    private double pTurn = 0.045, iTurn = 0, dTurn = 0.05;
//
//    private boolean targetFound = false;
//    private double aprilTagDrive = 0;
//    private double strafe = 0;
//    private double turn = 0;
//
//    private double initWrist = -140;
//    private ServoDiffyWrist diffyWrist;
//    private Elbow arm2;
//    private Arm1 arm1;
//    private CRServo intake;
//    private Servo door;
//
//    public static double backdropWrist = -85;
//    public static double purpleWrist = -50;
//    public static double intakingWrist = -50;
//
//    PIDFController speedController = new PIDFController(pX, iX, dX, 0);
//    PIDFController strafeController = new PIDFController(pY, iY, dY, 0);
//    PIDFController turnController = new PIDFController(pTurn, iTurn, dTurn, 0);
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        aprilTag = new AprilTagProcessor.Builder().build();
//        aprilTag.setDecimation(2);
//        setManualExposure(5, 250);  // Use low exposure time to reduce motion blur
//
//        propPipeline = new PropPipeline();
//
//        portal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(aprilTag)
//                .addProcessor(propPipeline)
//                .enableLiveView(true)
//                .setAutoStopLiveView(true)
//                .build();
//
//        FtcDashboard.getInstance().startCameraStream(propPipeline, 30);
//
////        initAprilTag();
//
//        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
//        leftBackDrive  = hardwareMap.get(DcMotor.class, "bl");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
//
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
//
////        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
//
//        // Wait for driver to press start
//        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
//        telemetry.addData(">", "Touch Play to start OpMode");
//
////        IMU imu = hardwareMap.get(IMU.class, "imu");
////        imu.initialize(
////                new IMU.Parameters(
//////                        new RevHubOrientationOnRobot(RevHubOrientationOnRobot.xyzOrientation(Math.toDegrees(0),Math.toDegrees(0),Math.toDegrees(-90)))
////                        new RevHubOrientationOnRobot(
////                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
////                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
////                        )
////                )
////        );
//
//
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        diffyWrist = new ServoDiffyWrist(hardwareMap);
//        arm1 = (new Arm1(hardwareMap));
//        arm2 = new Elbow(hardwareMap);
//
//        diffyWrist.runToProfile(initWrist, 0);
//
//        DcMotor left_lift = hardwareMap.get(DcMotor.class, "left_lift");
//        DcMotor right_lift = hardwareMap.get(DcMotor.class, "right_lift");
//        DcMotor elbow = hardwareMap.get(DcMotor.class, "elbow");
//        intake = hardwareMap.get(CRServo.class, "intake");
//        door = hardwareMap.get(Servo.class, "door");
//
//        Pose2d firstTile = new Pose2d(15, -6, Math.toRadians(0));
//
//        Pose2d stackIntakingPos = new Pose2d(52,-3,Math.toRadians(90));
//
//        Pose2d spike3Avoid = new Pose2d(48, -8,Math.toRadians(180));
//        Pose2d spike2Avoid = new Pose2d(48, -25, Math.toRadians(105));
//        Pose2d spike1Avoid = new Pose2d(30.5, -6, Math.toRadians(90));
//        Pose2d MiddleTile = new Pose2d(52,70, Math.toRadians(90));
//        Pose2d spike3 = new Pose2d(43, -14, Math.toRadians(180));
//        Pose2d spike2 = new Pose2d(52, -6, Math.toRadians(180));
//        Pose2d spike1 = new Pose2d(32, -2, Math.toRadians(90));
//        Pose2d boardRight = new Pose2d(32.5, 72.3, Math.toRadians(90));
//        Pose2d boardMiddle = new Pose2d(23.5, 72.3, Math.toRadians(90));
//        Pose2d boardLeft = new Pose2d(15.2, 72.3, Math.toRadians(90));
//        Pose2d park = new Pose2d(52, 86, Math.toRadians(90));
//        Pose2d aprilTagPose = new Pose2d(23, 70, Math.toRadians(90));
//
//        double waitTime = 3;
//
//        TrajectorySequence linetoFirstTile = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .lineToLinearHeading(firstTile)
//                .build();
//        TrajectorySequence toSpike1 = drive.trajectorySequenceBuilder(firstTile)
//                .lineToLinearHeading(spike1)
//                .build();
//        TrajectorySequence toSpike2 = drive.trajectorySequenceBuilder(firstTile)
//                .lineToLinearHeading(spike2)
//                .build();
//        TrajectorySequence toSpike3 = drive.trajectorySequenceBuilder(firstTile)
//                .lineToLinearHeading(spike3)
//                .build();
//        TrajectorySequence avoid1 = drive.trajectorySequenceBuilder(spike1)
//                .lineToLinearHeading(spike1Avoid)
//                .build();
//        TrajectorySequence avoid2 = drive.trajectorySequenceBuilder(spike2)
//                .lineToLinearHeading(spike2Avoid)
//                .build();
//        TrajectorySequence avoid3 = drive.trajectorySequenceBuilder(spike3)
//                .lineToLinearHeading(spike3Avoid)
//                .build();
//        TrajectorySequence toNextLeft = drive.trajectorySequenceBuilder(spike1Avoid)
//                .lineToLinearHeading(stackIntakingPos)
//                .build();
//        TrajectorySequence toNextCenter = drive.trajectorySequenceBuilder(spike2Avoid)
//                .lineToLinearHeading(stackIntakingPos)
//                .build();
//        TrajectorySequence toNextRight = drive.trajectorySequenceBuilder(spike3Avoid)
//                .lineToLinearHeading(stackIntakingPos)
//                .build();
//
//        while (opModeInInit()) {
//
//            if (gamepad2.a) {
//                left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                left_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }
//
//            if(gamepad2.b) {
//                elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }
//
//            if(gamepad2.dpad_up) {
//                waitTime += 0.5;
//                sleep(350);
//            }
//            if(gamepad2.dpad_down) {
//                waitTime -= 0.5;
//                sleep(350);
//            }
//            if(gamepad2.right_bumper) {
//                door.setPosition(0.1);
//            }
//            if(gamepad2.left_bumper) {
//                door.setPosition(0.90);
//            }
//
//            if(waitTime < 0)
//                waitTime = 0;
//            if(waitTime > 4)
//                waitTime = 4;
//
//            left_lift.setPower(-gamepad2.right_stick_y);
//            right_lift.setPower(-gamepad2.right_stick_y);
//            elbow.setPower(gamepad2.left_stick_y);
//            //wrist.setPosition(0.63);
//
////            telemetry.addData("Parallel: ", parallelEncoder.getCurrentPosition());
////            telemetry.addData("Perpendicular: ", perpendicularEncoder.getCurrentPosition());
//            telemetry.addData("Wait", waitTime);
//            telemetry.addData("Left Lift Encoder", left_lift.getCurrentPosition());
//            telemetry.addData("Right Lift Encoder", right_lift.getCurrentPosition());
//            telemetry.addData("Elbow Encoder", elbow.getCurrentPosition());
//            telemetry.addData("Location", propPipeline.getLocation());
////            telemetry.addData("imu", imu.getRobotAngularVelocity(AngleUnit.DEGREES));
//
//            telemetry.update();
//        }
//
//
//        TrajectorySequence toMiddle = drive.trajectorySequenceBuilder(stackIntakingPos)
//                .waitSeconds(waitTime)
//                .lineToLinearHeading(MiddleTile)
//                .build();
//        TrajectorySequence toAprilTag = drive.trajectorySequenceBuilder(MiddleTile)
//                .lineToLinearHeading(aprilTagPose)
//                .build();
////        TrajectorySequence toBoardLeft = drive.trajectorySequenceBuilder(MiddleTile)
////                .lineToLinearHeading(boardLeft)
////                .build();
////        TrajectorySequence toBoardCenter = drive.trajectorySequenceBuilder(MiddleTile)
////                .lineToLinearHeading(boardMiddle)
////                .build();
////        TrajectorySequence toBoardRight = drive.trajectorySequenceBuilder(MiddleTile)
////                .lineToLinearHeading(boardRight)
////                .build();
//        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(boardLeft)
//                .lineToLinearHeading(park)
//                .build();
//        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(boardMiddle)
//                .lineToLinearHeading(park)
//                .build();
//        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(boardRight)
//                .lineToLinearHeading(park)
//                .build();
//
//        boolean ButtonXBlock = false;
//        double wristservoposition = 0.63;
//        //wrist.setPosition(wristservoposition);
//
//
//
//        waitForStart();
//
//        if (!isStopRequested()) {
//
//            Location location = propPipeline.getLocation();
//
//            //wrist.setPosition(0.05);
////            door.setPosition(0.9);
////            arm1.ArmToPos(-2000, 0.5);
////            arm2.ArmToPos(210, 1);
////            diffyWrist.runToProfile(purpleWrist, 0);
//
//            if (location == LEFT) {
//                // Movements for left spot
//
//                telemetry.addData("Position", "Left");
//                telemetry.update();
//
//                drive.followTrajectorySequence(linetoFirstTile);
//                door.setPosition(0.9);
//                arm1.ArmToPos(-2000, 0.5);
//                arm2.ArmToPos(210, 1);
//                diffyWrist.runToProfile(purpleWrist, 0);
//                drive.followTrajectorySequence(toSpike1);
//                diffyWrist.runToProfile(purpleWrist, 0);
//                //wrist.setPosition(0.24);
//                sleep(500);
//                door.setPosition(0.1);
//                sleep(200);
//                arm2.ArmToPos(0,1);
//                sleep(500);
//                door.setPosition(0.90);
//                drive.followTrajectorySequence(avoid1);
//                drive.followTrajectorySequence(toNextLeft);
//                arm1.ArmToPos(-1840,1);
//                sleep(1000);
//                //cycle
////                arm2.ArmToPos(-2130,1);
////                sleep(2500);
////                diffyWrist.runToProfile(intakingWrist, -235);
////                sleep(2000);
////                intake.setPower(-1);
////                sleep(1000);
////                intake.setPower(0);
//                arm2.ArmToPos(100,1);
//                //wrist.setPosition(0.3);
//                diffyWrist.runToProfile(backdropWrist, 0);
//                drive.followTrajectorySequence(toMiddle);
//                drive.followTrajectorySequence(toAprilTag);
//                alignToAprilTags();
//                drive.setPoseEstimate(aprilTagPose);
//                TrajectorySequence toBoardLeft = drive.trajectorySequenceBuilder(aprilTagPose)
//                        .lineToLinearHeading(boardLeft)
//                        .build();
//                drive.followTrajectorySequence(toBoardLeft);
//                arm1.ArmToPos(-680, 0.5);
//                //wrist.setPosition(0.8);
//                arm2.ArmToPos(-830, 0.65);
//                intake.setPower(-1);
//                sleep(500);
//                intake.setPower(0);
//                sleep(1000);
//                sleep(1000);
//                door.setPosition(0.1);
//                sleep(100);
////                arm2.ArmToPos(-1100, 0.5);
////                sleep(1000);
//                arm1.ArmToPos(-1850, 0.5);
//                sleep(1000);
//                drive.followTrajectorySequence(leftPark);
//                arm2.ArmToPos(0,1);
//
//            }
//            if (location == CENTER) {
//                // Movements for center spot
//                telemetry.addData("Position", "Center");
//                telemetry.update();
//
//                drive.followTrajectorySequence(linetoFirstTile);
//                door.setPosition(0.9);
//                arm1.ArmToPos(-2000, 0.5);
//                arm2.ArmToPos(210, 1);
//                diffyWrist.runToProfile(purpleWrist, 0);
//                drive.followTrajectorySequence(toSpike2);
//                sleep(500);
//                door.setPosition(0.1);
//                sleep(200);
//                arm2.ArmToPos(0,1);
//                sleep(500);
//                door.setPosition(0.90);
//                drive.followTrajectorySequence(avoid2);
//                drive.followTrajectorySequence(toNextCenter);
//                arm1.ArmToPos(-1840,1);
//                sleep(1000);
//                //cycle
////                arm2.ArmToPos(-2130,0.7);
////                sleep(2500);
////                arm2.ArmToPos(100,0.7);
////                diffyWrist.runToProfile(intakingWrist, -235);
////                sleep(2000);
////                intake.setPower(-1);
//                sleep(1000);
//                intake.setPower(0);
//                sleep(1000);
//                //wrist.setPosition(0.3);
//                diffyWrist.runToProfile(purpleWrist, 0);
//                drive.followTrajectorySequence(toMiddle);
//                drive.followTrajectorySequence(toAprilTag);
//                alignToAprilTags();
//                drive.setPoseEstimate(aprilTagPose);
//                TrajectorySequence toBoardCenter = drive.trajectorySequenceBuilder(aprilTagPose)
//                        .lineToLinearHeading(boardMiddle)
//                        .build();
//                drive.followTrajectorySequence(toBoardCenter);
//                arm1.ArmToPos(-680, 0.5);
//                //wrist.setPosition(0.8);
//                diffyWrist.runToProfile(backdropWrist, 0);
//                arm2.ArmToPos(-830, 65);
//                intake.setPower(-1);
//                sleep(500);
//                intake.setPower(0);
//                sleep(1000);
//                door.setPosition(0.1);
//                sleep(100);
////                arm2.ArmToPos(-1100, 0.5);
////                sleep(1000);
//                arm1.ArmToPos(-1850, 0.5);
//                sleep(1000);
//                drive.followTrajectorySequence(centerPark);
//                arm2.ArmToPos(0,1);
//            }
//            if (location == RIGHT) {
//                // Movements for Right spot
//
//                telemetry.addData("Position", "Right");
//                telemetry.update();
//
//                drive.followTrajectorySequence(linetoFirstTile);
//                door.setPosition(0.9);
//                arm1.ArmToPos(-2000, 0.5);
//                arm2.ArmToPos(210, 1);
//                diffyWrist.runToProfile(purpleWrist, 0);
//                drive.followTrajectorySequence(toSpike3);
//                door.setPosition(0.1);
//                sleep(200);
//                arm2.ArmToPos(0,1);
//                door.setPosition(0.90);
//                drive.followTrajectorySequence(avoid3);
//                drive.followTrajectorySequence(toNextRight);
//////               arm1.ArmToPos(-1840,1);
//////               stackIntake();
////                retractArms();
////                arm2.ArmToPos(-2130,0.6);
////                sleep(2500);
////                diffyWrist.runToProfile(intakingWrist, -235);
////                sleep(2000);
////                intake.setPower(-1);
////                sleep(1000);
////                intake.setPower(0);
////                sleep(2000);
//                arm2.ArmToPos(100,0.6);
//                //wrist.setPosition(0.3);
//                diffyWrist.runToProfile(purpleWrist, 0);
//                drive.followTrajectorySequence(toMiddle);
////                sleep(500);
//                drive.followTrajectorySequence(toAprilTag);
//                alignToAprilTags();
//                drive.setPoseEstimate(aprilTagPose);
//                TrajectorySequence toBoardRight = drive.trajectorySequenceBuilder(aprilTagPose)
//                        .lineToLinearHeading(boardRight)
//                        .build();
//                drive.followTrajectorySequence(toBoardRight);
//                arm1.ArmToPos(-680, 0.65);
////                wrist.setPosition(0.8);
//                diffyWrist.runToProfile(backdropWrist, 0);
//                arm2.ArmToPos(-830, 0.65);
//                intake.setPower(-1);
//                sleep(500);
//                intake.setPower(0);
//                sleep(1000);
//                door.setPosition(0.1);
//                sleep(100);
////                arm2.ArmToPos(-1100, 0.5);
////                sleep(1000);
//                arm1.ArmToPos(-1850, 0.5);
//                sleep(1000);
//                drive.followTrajectorySequence(rightPark);
//                arm2.ArmToPos(0,1);
//            }
//            door.setPosition(0.90);
////            drive.followTrajectorySequence(toPrepare);
//            sleep(30000);
//        }
//    }
//
//    private void    setManualExposure(int exposureMS, int gain) {
//        // Wait for the camera to be open, then use the controls
//
//        if (portal == null) {
//            return;
//        }
//
//        // Make sure camera is streaming before we try to set the exposure controls
//        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addData("Camera", "Waiting");
//            telemetry.update();
//            while (!isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                sleep(20);
//            }
//            telemetry.addData("Camera", "Ready");
//            telemetry.update();
//        }
//
//        // Set camera controls unless we are stopping.
//        if (!isStopRequested())
//        {
//            ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
//            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//                exposureControl.setMode(ExposureControl.Mode.Manual);
//                sleep(50);
//            }
//            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
//            sleep(20);
//            GainControl gainControl = portal.getCameraControl(GainControl.class);
//            gainControl.setGain(gain);
//            sleep(20);
//        }
//    }
//
//    private void alignToAprilTags() {
//
//        if (!isStopRequested()) {
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
//
//            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
//            if (targetFound) {
//                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
//                turn = turnController.calculate(0, desiredTag.ftcPose.pitch);
//                strafe = (strafeController.calculate(0, desiredTag.ftcPose.elevation));
//                aprilTagDrive = speedController.calculate(18.5, desiredTag.ftcPose.range);
//
//                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", aprilTagDrive, strafe, turn);
//                telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
//                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
//                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
//                telemetry.addData("elevation", "%3.0f degrees", desiredTag.ftcPose.elevation);
//                telemetry.addData("pitch", "%3.0f degrees", desiredTag.ftcPose.pitch);
//
//            }
//            telemetry.update();
//        }
//
//        while (Math.abs(aprilTagDrive) > 0.1) {
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
//                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
//                turn = turnController.calculate(0, desiredTag.ftcPose.pitch);
//                strafe = (strafeController.calculate(0, desiredTag.ftcPose.elevation));
//                aprilTagDrive = speedController.calculate(18.5, desiredTag.ftcPose.range);
//
//                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", aprilTagDrive, strafe, turn);
//                telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
//                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
//                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
//                telemetry.addData("elevation", "%3.0f degrees", desiredTag.ftcPose.elevation);
//                telemetry.addData("pitch", "%3.0f degrees", desiredTag.ftcPose.pitch);
//
//            }
//            telemetry.update();
//
//            // Apply desired axes motions to the drivetrain.
//            moveRobot(aprilTagDrive, strafe, turn);
//            sleep(10);
//        }
//    }
//
//    public void moveRobot(double x, double y, double yaw) {
//        // Calculate wheel powers.
//        double leftFrontPower    =  x -y -yaw;
//        double rightFrontPower   =  x +y +yaw;
//        double leftBackPower     =  x +y -yaw;
//        double rightBackPower    =  x -y +yaw;
//
//        // Normalize wheel powers to be less than 1.0
//        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//        max = Math.max(max, Math.abs(leftBackPower));
//        max = Math.max(max, Math.abs(rightBackPower));
//
//        if (max > 1.0) {
//            leftFrontPower /= max;
//            rightFrontPower /= max;
//            leftBackPower /= max;
//            rightBackPower /= max;
//        }
//
//        // Send powers to the wheels.
//        leftFrontDrive.setPower(leftFrontPower);
//        rightFrontDrive.setPower(rightFrontPower);
//        leftBackDrive.setPower(leftBackPower);
//        rightBackDrive.setPower(rightBackPower);
//    }
//
//    public void stackIntake() {
//        intake.setPower(1);
//        arm2.ArmToPos(-1000, 1);
//        while (arm2.isBusy()){
//            arm2.updateElbow();
//            telemetry.addData("moving", 0);
//        }
//        diffyWrist.runToProfile(0, -180);
//        arm2.ArmToPos(-2100, 0.7);
//        diffyWrist.runToProfile(-20, -180);
//        while (arm2.isBusy()){
//            arm2.updateElbow();
//            telemetry.addData("moving", 0);
//        }
//        intake.setPower(0);
//    }
//
//    public void retractArms() {
//        door.setPosition(0.9);
//        arm1.ArmToPos(-2000, 0.7);
//        arm2.ArmToPos(138, 1);
//        diffyWrist.runToProfile(purpleWrist, 0);
//    }
//
//}