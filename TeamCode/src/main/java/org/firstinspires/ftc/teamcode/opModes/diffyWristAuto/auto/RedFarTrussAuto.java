//package org.firstinspires.ftc.teamcode.opModes.diffyWristAuto.auto;
//
//import static org.firstinspires.ftc.teamcode.subsystems.vision.old.PropPipeline.Location.CENTER;
//import static org.firstinspires.ftc.teamcode.subsystems.vision.old.PropPipeline.Location.LEFT;
//import static org.firstinspires.ftc.teamcode.subsystems.vision.old.PropPipeline.Location.RIGHT;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
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
//import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.opmode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.subsystems.vision.old.PropPipeline;
//import org.firstinspires.ftc.teamcode.subsystems.vision.old.PropPipeline.Location;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.concurrent.TimeUnit;
//
////@Disabled
//@Autonomous(name = "Red Far Truss 2+0")
//public class RedFarTrussAuto extends LinearOpMode {
//
//    private PropPipeline propPipeline;
//    private VisionPortal portal;
//    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
//    private AprilTagDetection desiredTag = null;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        aprilTag = new AprilTagProcessor.Builder().build();
//        aprilTag.setDecimation(2);
//        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
//
//        propPipeline = new PropPipeline();
//
//        portal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//
//                .addProcessor(aprilTag)
//                .addProcessor(propPipeline)
//                .enableLiveView(true)
//                .setAutoStopLiveView(true)
//                .build();
//
//        FtcDashboard.getInstance().startCameraStream(propPipeline, 30);
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        Arm1 arm1 = (new Arm1(hardwareMap));
//        Elbow arm2 = new Elbow(hardwareMap);
//
//        Servo wrist = hardwareMap.get(Servo.class, "wrist");
//        DcMotor left_lift = hardwareMap.get(DcMotor.class, "left_lift");
//        DcMotor right_lift = hardwareMap.get(DcMotor.class, "right_lift");
//        DcMotor elbow = hardwareMap.get(DcMotor.class, "elbow");
//        CRServo intake = hardwareMap.get(CRServo.class, "intake");
//        Servo door = hardwareMap.get(Servo.class, "door");
//
//        Pose2d firstTile = new Pose2d(15, -2, Math.toRadians(0));
//        Pose2d nextTile = new Pose2d(1.5,-2,Math.toRadians(-90));
//        Pose2d MiddleTile = new Pose2d(1.5, -75, Math.toRadians(0));
//        Pose2d spike1 = new Pose2d(28, 11, Math.toRadians(0));
//        Pose2d spike2 = new Pose2d(32, -4, Math.toRadians(0));
//        Pose2d spike3 = new Pose2d(30.5, -6.75, Math.toRadians(-90));
//        Pose2d boardLeft = new Pose2d(16.5, -75, Math.toRadians(-90));
//        Pose2d boardMiddle = new Pose2d(25, -75, Math.toRadians(-90));
//        Pose2d boardRight = new Pose2d(33.5, -75, Math.toRadians(-90));
//        Pose2d park = new Pose2d(0, -83, Math.toRadians(90));
//
//        TrajectorySequence lineToFirstTile = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
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
//        TrajectorySequence toFirstLeft = drive.trajectorySequenceBuilder(spike1)
//                .lineToLinearHeading(firstTile)
//                .build();
//        TrajectorySequence toFirstCenter = drive.trajectorySequenceBuilder(spike2)
//                .lineToLinearHeading(firstTile)
//                .build();
//        TrajectorySequence toFirstRight = drive.trajectorySequenceBuilder(spike3)
//                .lineToLinearHeading(firstTile)
//                .build();
//        TrajectorySequence toNextTile = drive.trajectorySequenceBuilder(firstTile)
//                .lineToLinearHeading(nextTile)
//                .build();
//        TrajectorySequence toMiddle = drive.trajectorySequenceBuilder(nextTile)
//                .lineToLinearHeading(MiddleTile)
//                .build();
//        TrajectorySequence toBoardLeft = drive.trajectorySequenceBuilder(MiddleTile)
//                .lineToLinearHeading(boardLeft)
//                .build();
//        TrajectorySequence toBoardCenter = drive.trajectorySequenceBuilder(MiddleTile)
//                .lineToLinearHeading(boardMiddle)
//                .build();
//        TrajectorySequence toBoardRight = drive.trajectorySequenceBuilder(MiddleTile)
//                .lineToLinearHeading(boardRight)
//                .build();
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
//        wrist.setPosition(wristservoposition);
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
//            left_lift.setPower(-gamepad2.right_stick_y);
//            right_lift.setPower(-gamepad2.right_stick_y);
//            elbow.setPower(gamepad2.left_stick_y);
//            wrist.setPosition(0.63);
//
//            telemetry.addData("Left Lift Encoder", left_lift.getCurrentPosition());
//            telemetry.addData("Right Lift Encoder", right_lift.getCurrentPosition());
//            telemetry.addData("Elbow Encoder", elbow.getCurrentPosition());
//            telemetry.addData("Location", propPipeline.getLocation());
//            telemetry.update();
//        }
//
//        waitForStart();
//
//        if (!isStopRequested()) {
//
//            Location location = propPipeline.getLocation();
//
//            wrist.setPosition(0.05);
//            door.setPosition(0.75);
//            arm1.ArmToPos(-2000, 0.5);
//            arm2.ArmToPos(210, 1);
//
//            if (location == LEFT) {
//                // Movements for left spot
//                telemetry.addData("Position", "Left");
//                telemetry.update();
//
//                drive.followTrajectorySequence(lineToFirstTile);
//                drive.followTrajectorySequence(toSpike1);
//                wrist.setPosition(0.24);
//                sleep(500);
//
//                wrist.setPosition(0.24);
//
//                door.setPosition(0.1);
//                sleep(200);
//                arm2.ArmToPos(0,1);
//                sleep(500);
//                door.setPosition(0.95);
//                drive.followTrajectorySequence(toFirstLeft);
//                drive.followTrajectorySequence(toNextTile);
//                arm2.ArmToPos(200,1);
//                wrist.setPosition(0.3);
//                drive.followTrajectorySequence(toMiddle);
//                drive.followTrajectorySequence(toBoardLeft);
//                arm1.ArmToPos(-664, 0.5);
//                wrist.setPosition(0.8);
//                arm2.ArmToPos(-812, 0.65);
//                intake.setPower(-1);
//                sleep(500);
//                intake.setPower(0);
//                sleep(1000);
//                door.setPosition(0.1);
//                sleep(749);
//                arm2.ArmToPos(-900, 0.5);
//                sleep(500);
//                door.setPosition(0.95);
//                sleep(1000);
//                arm1.ArmToPos(-1850, 0.5);
//                drive.followTrajectorySequence(leftPark);
//                arm2.ArmToPos(0,1);
//
//            }
//            if (location == CENTER) {
//                // Movements for center spot
//                telemetry.addData("Position", "Center");
//                telemetry.update();
//
//                drive.followTrajectorySequence(lineToFirstTile);
//                drive.followTrajectorySequence(toSpike2);
//                sleep(500);
//
//                wrist.setPosition(0.24);
//
//                door.setPosition(0.1);
//                sleep(200);
//                arm2.ArmToPos(0,1);
//                sleep(500);
//                door.setPosition(0.95);
//                drive.followTrajectorySequence(toFirstCenter);
//                drive.followTrajectorySequence(toNextTile);
//                arm2.ArmToPos(200,1);
//                wrist.setPosition(0.3);
//                drive.followTrajectorySequence(toMiddle);
//                drive.followTrajectorySequence(toBoardCenter);
//                arm1.ArmToPos(-664, 0.5);
//                wrist.setPosition(0.8);
//                arm2.ArmToPos(-812, 0.65);
//                intake.setPower(-1);
//                sleep(500);
//                intake.setPower(0);
//                sleep(1000);
//                door.setPosition(0.1);
//                sleep(749);
//                arm2.ArmToPos(-900, 0.5);
//                sleep(500);
//                door.setPosition(0.95);
//                sleep(1000);
//                arm1.ArmToPos(-1850, 0.5);
//                drive.followTrajectorySequence(centerPark);
//                arm2.ArmToPos(0,1);
//
//            }
//            if (location == RIGHT) {
//                // Movements for Right spot
//                telemetry.addData("Position", "Right");
//                telemetry.update();
//
//                drive.followTrajectorySequence(lineToFirstTile);
//                drive.followTrajectorySequence(toSpike3);
//                sleep(500);
//
//                wrist.setPosition(0.24);
//
//                door.setPosition(0.1);
//                sleep(200);
//                arm2.ArmToPos(0,1);
//                sleep(500);
//                door.setPosition(0.95);
//                drive.followTrajectorySequence(toFirstRight);
//                drive.followTrajectorySequence(toNextTile);
//                arm2.ArmToPos(200,1);
//                wrist.setPosition(0.3);
//                drive.followTrajectorySequence(toMiddle);
//                drive.followTrajectorySequence(toBoardRight);
//                arm1.ArmToPos(-664, 0.5);
//                wrist.setPosition(0.8);
//                arm2.ArmToPos(-812, 0.65);
//                intake.setPower(-1);
//                sleep(500);
//                intake.setPower(0);
//                sleep(1000);
//                door.setPosition(0.1);
//                sleep(749);
//                arm2.ArmToPos(-900, 0.5);
//                sleep(500);
//                door.setPosition(0.95);
//                sleep(1000);
//                arm1.ArmToPos(-1850, 0.5);
//                drive.followTrajectorySequence(rightPark);
//                arm2.ArmToPos(0,1);
//            }
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
//}