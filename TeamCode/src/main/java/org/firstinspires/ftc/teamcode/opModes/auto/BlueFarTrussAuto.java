package org.firstinspires.ftc.teamcode.opModes.auto;

import static org.firstinspires.ftc.teamcode.subsystems.vision.myPropPipeline.Location.CENTER;
import static org.firstinspires.ftc.teamcode.subsystems.vision.myPropPipeline.Location.LEFT;
import static org.firstinspires.ftc.teamcode.subsystems.vision.myPropPipeline.Location.RIGHT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.Arm1;
import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.ServoArm;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.vision.myPropPipeline;
import org.firstinspires.ftc.teamcode.subsystems.vision.myPropPipeline.Location;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

//@Disabled
@Autonomous(name = "Blue Far Truss 2+0")
public class BlueFarTrussAuto extends LinearOpMode {

    private myPropPipeline propPipeline;
    private VisionPortal portal;
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;

    @Override
    public void runOpMode() throws InterruptedException {

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);
        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        propPipeline = new myPropPipeline();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))

                .addProcessor(aprilTag)
                .addProcessor(propPipeline)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(propPipeline, 30);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Arm1 arm1 = (new Arm1(hardwareMap));
        ServoArm arm2 = new ServoArm(hardwareMap);

        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        DcMotor left_lift = hardwareMap.get(DcMotor.class, "left_lift");
        DcMotor right_lift = hardwareMap.get(DcMotor.class, "right_lift");
        CRServo leftArm = hardwareMap.get(CRServo.class, "leftArm");
        CRServo rightArm = hardwareMap.get(CRServo.class, "rightArm");
        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        Servo door = hardwareMap.get(Servo.class, "door");

        Pose2d firstTile = new Pose2d(15, -4, Math.toRadians(0));
        Pose2d nextTile = new Pose2d(1.5,-4,Math.toRadians(90));
        Pose2d MiddleTile = new Pose2d(1.5,82, Math.toRadians(90));
        Pose2d spike3 = new Pose2d(28, -13, Math.toRadians(0));
        Pose2d spike2 = new Pose2d(31, -4, Math.toRadians(0));
        Pose2d spike1 = new Pose2d(30.5, 6, Math.toRadians(90));
        Pose2d boardRight = new Pose2d(34.5, 72.5, Math.toRadians(90));
        Pose2d boardMiddle = new Pose2d(26, 72.5, Math.toRadians(90));
        Pose2d boardLeft = new Pose2d(18.5, 72.5, Math.toRadians(90));
        Pose2d park = new Pose2d(0, 86, Math.toRadians(90));

        TrajectorySequence lineToFirstTile = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(firstTile)
                .build();
        TrajectorySequence toSpike1 = drive.trajectorySequenceBuilder(firstTile)
                .lineToLinearHeading(spike1)
                .build();
        TrajectorySequence toSpike2 = drive.trajectorySequenceBuilder(firstTile)
                .lineToLinearHeading(spike2)
                .build();
        TrajectorySequence toSpike3 = drive.trajectorySequenceBuilder(firstTile)
                .lineToLinearHeading(spike3)
                .build();
        TrajectorySequence toFirstLeft = drive.trajectorySequenceBuilder(spike1)
                .lineToLinearHeading(firstTile)
                .build();
        TrajectorySequence toFirstCenter = drive.trajectorySequenceBuilder(spike2)
                .lineToLinearHeading(firstTile)
                .build();
        TrajectorySequence toFirstRight = drive.trajectorySequenceBuilder(spike3)
                .lineToLinearHeading(firstTile)
                .build();
        TrajectorySequence toNextTile = drive.trajectorySequenceBuilder(firstTile)
                .lineToLinearHeading(nextTile)
                .build();
        TrajectorySequence toMiddle = drive.trajectorySequenceBuilder(nextTile)
                .lineToLinearHeading(MiddleTile)
                .build();
        TrajectorySequence toBoardLeft = drive.trajectorySequenceBuilder(MiddleTile)
                .lineToLinearHeading(boardLeft)
                .build();
        TrajectorySequence toBoardCenter = drive.trajectorySequenceBuilder(MiddleTile)
                .lineToLinearHeading(boardMiddle)
                .build();
        TrajectorySequence toBoardRight = drive.trajectorySequenceBuilder(MiddleTile)
                .lineToLinearHeading(boardRight)
                .build();
        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(boardLeft)
                .lineToLinearHeading(park)
                .build();
        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(boardMiddle)
                .lineToLinearHeading(park)
                .build();
        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(boardRight)
                .lineToLinearHeading(park)
                .build();

        boolean ButtonXBlock = false;
        double wristservoposition = 0.63;
        wrist.setPosition(wristservoposition);

        while (opModeInInit()) {

            if (gamepad2.a) {
                left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                left_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            left_lift.setPower(-gamepad2.right_stick_y);
            right_lift.setPower(-gamepad2.right_stick_y);
            leftArm.setPower(-gamepad2.left_stick_y);
            rightArm.setPower(gamepad2.left_stick_y);

            telemetry.addData("Left Lift Encoder", left_lift.getCurrentPosition());
            telemetry.addData("Right Lift Encoder", right_lift.getCurrentPosition());
            telemetry.addData("Servo Arm", arm2.getLocation());

            telemetry.addData("Location", propPipeline.getLocation());
            telemetry.update();
        }

        waitForStart();

        if (!isStopRequested()) {

            Location location = propPipeline.getLocation();

            wrist.setPosition(0.3);
            door.setPosition(0.75);
            arm1.ArmToPos(-1750, 1);

            if (location == LEFT) {
                // Movements for left spot

                telemetry.addData("Position", "Left");
                telemetry.update();

                drive.followTrajectorySequence(lineToFirstTile);
//
//                sleep(1000);

                drive.followTrajectorySequence(toSpike1);

                sleep(200);

                wrist.setPosition(0.21);

                arm1.ArmToPos(-2000, 1);

                arm2.runToProfile(-7.5);
                while ((arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "Left");
                    telemetry.addData("Arm2", arm2.getLocation());
                    telemetry.addData("Arm2 State", arm2.isBusy());
                    telemetry.update();
                }

                door.setPosition(0.1);

                sleep(1000);

                arm2.runToProfile(0);
                while ((arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "Left");
                    telemetry.addData("Arm2", arm2.getLocation());
                    telemetry.addData("Arm2 State", arm2.isBusy());
                    telemetry.update();
                }

                door.setPosition(0.95);

                drive.followTrajectorySequence(toFirstLeft);

                sleep(500);

                drive.followTrajectorySequence(toNextTile);

                sleep(500);

                drive.followTrajectorySequence(toMiddle);

                sleep(500);

                drive.followTrajectorySequence(toBoardLeft);

                arm2.runToProfile(0);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "Left");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

                arm1.ArmToPos(-600, 0.5);

                wrist.setPosition(0.45);

                arm2.runToProfile(100);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "Left");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

                sleep(750);


                intake.setPower(-1);

                sleep(750);

                intake.setPower(0);

                door.setPosition(0.1);

                sleep(700);

//                arm2.runToProfile(120);
//                while( (arm2.isBusy()) && !isStopRequested()) {
//                    arm2.updateServoArm();
//                    telemetry.addData("Position", "Left");
//                    telemetry.addData("Arm2" , arm2.getLocation());
//                    telemetry.addData("Arm2 State" , arm2.isBusy());
//                    telemetry.update();
//                }

                door.setPosition(0.95);

                arm1.ArmToPos(-1100, 1);

                arm2.runToProfile(0);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "Left");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

//                arm1.ArmToPos(0,0.5);

                arm2.runToProfile(0);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "Left");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

                drive.followTrajectorySequence(leftPark);

            }
            if (location == CENTER) {
                // Movements for center spot
                telemetry.addData("Position", "Center");
                telemetry.update();

                drive.followTrajectorySequence(lineToFirstTile);
//
//                sleep(1000);

                drive.followTrajectorySequence(toSpike2);

                sleep(200);

                wrist.setPosition(0.21);

                arm1.ArmToPos(-2000, 1);

                arm2.runToProfile(-7.5);
                while ((arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "Center");
                    telemetry.addData("Arm2", arm2.getLocation());
                    telemetry.addData("Arm2 State", arm2.isBusy());
                    telemetry.update();
                }

                door.setPosition(0.1);

                sleep(1000);

                arm2.runToProfile(0);
                while ((arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "Center");
                    telemetry.addData("Arm2", arm2.getLocation());
                    telemetry.addData("Arm2 State", arm2.isBusy());
                    telemetry.update();
                }

                door.setPosition(0.95);

                drive.followTrajectorySequence(toFirstCenter);

                sleep(500);

                drive.followTrajectorySequence(toNextTile);

                sleep(500);

                drive.followTrajectorySequence(toMiddle);

                sleep(500);

                drive.followTrajectorySequence(toBoardCenter);

                arm2.runToProfile(0);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "Center");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

                arm1.ArmToPos(-600, 0.5);

                wrist.setPosition(0.45);

                arm2.runToProfile(100);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "Center");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

                sleep(750);

                intake.setPower(-1);

                sleep(750);

                intake.setPower(0);

                door.setPosition(0.1);

                sleep(700);

//                arm2.runToProfile(120);
//                while( (arm2.isBusy()) && !isStopRequested()) {
//                    arm2.updateServoArm();
//                    telemetry.addData("Position", "Center");
//                    telemetry.addData("Arm2" , arm2.getLocation());
//                    telemetry.addData("Arm2 State" , arm2.isBusy());
//                    telemetry.update();
//                }

                door.setPosition(0.95);

                arm1.ArmToPos(-1100, 1);

                arm2.runToProfile(0);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "Center");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

//                arm1.ArmToPos(0,0.5);

                arm2.runToProfile(0);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "Center");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

                drive.followTrajectorySequence(centerPark);

            }
            if (location == RIGHT) {
                // Movements for Right spot

                telemetry.addData("Position", "Right");
                telemetry.update();

                drive.followTrajectorySequence(lineToFirstTile);
//
//                sleep(1000);

                drive.followTrajectorySequence(toSpike3);

                sleep(200);

                wrist.setPosition(0.21);

                arm1.ArmToPos(-2000, 1);

                arm2.runToProfile(-7.5);
                while ((arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "Right");
                    telemetry.addData("Arm2", arm2.getLocation());
                    telemetry.addData("Arm2 State", arm2.isBusy());
                    telemetry.update();
                }

                door.setPosition(0.1);

                sleep(1000);

                arm2.runToProfile(0);
                while ((arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "Right");
                    telemetry.addData("Arm2", arm2.getLocation());
                    telemetry.addData("Arm2 State", arm2.isBusy());
                    telemetry.update();
                }

                door.setPosition(0.95);

                drive.followTrajectorySequence(toFirstRight);

                sleep(500);

                drive.followTrajectorySequence(toNextTile);

                sleep(500);

                drive.followTrajectorySequence(toMiddle);

                sleep(500);

                drive.followTrajectorySequence(toBoardRight);

                arm2.runToProfile(0);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "Right");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

                arm1.ArmToPos(-600, 0.5);

                wrist.setPosition(0.45);

                arm2.runToProfile(100);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "Right");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

                sleep(750);


                intake.setPower(-1);

                sleep(750);

                intake.setPower(0);

                door.setPosition(0.1);

                sleep(700);

//                arm2.runToProfile(120);
//                while( (arm2.isBusy()) && !isStopRequested()) {
//                    arm2.updateServoArm();
//                    telemetry.addData("Position", "Right");
//                    telemetry.addData("Arm2" , arm2.getLocation());
//                    telemetry.addData("Arm2 State" , arm2.isBusy());
//                    telemetry.update();
//                }

                door.setPosition(0.95);

                arm1.ArmToPos(-1100, 1);

                arm2.runToProfile(0);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "Right");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

//                arm1.ArmToPos(0,0.5);

                arm2.runToProfile(0);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "Right");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

                drive.followTrajectorySequence(rightPark);
            }
            sleep(30000);
        }
    }

    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (portal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = portal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

}