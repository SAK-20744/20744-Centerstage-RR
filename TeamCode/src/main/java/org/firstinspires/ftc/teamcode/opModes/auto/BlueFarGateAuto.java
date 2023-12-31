package org.firstinspires.ftc.teamcode.opModes.auto;

import static org.firstinspires.ftc.teamcode.subsystems.vision.PropPipeline.Location.CENTER;
import static org.firstinspires.ftc.teamcode.subsystems.vision.PropPipeline.Location.LEFT;
import static org.firstinspires.ftc.teamcode.subsystems.vision.PropPipeline.Location.RIGHT;

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
import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.Elbow;
import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.ServoArm;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.vision.PropPipeline;
import org.firstinspires.ftc.teamcode.subsystems.vision.PropPipeline;
import org.firstinspires.ftc.teamcode.subsystems.vision.PropPipeline.Location;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

//@Disabled
@Autonomous(name = "Blue Far Gate 2+0")
public class BlueFarGateAuto extends LinearOpMode {

    private PropPipeline propPipeline;
    private VisionPortal portal;
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;

    @Override
    public void runOpMode() throws InterruptedException {

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);
        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        propPipeline = new PropPipeline();

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
        Elbow arm2 = new Elbow(hardwareMap);

        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        DcMotor left_lift = hardwareMap.get(DcMotor.class, "left_lift");
        DcMotor right_lift = hardwareMap.get(DcMotor.class, "right_lift");
        DcMotor elbow = hardwareMap.get(DcMotor.class, "elbow");
        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        Servo door = hardwareMap.get(Servo.class, "door");

        Pose2d firstTile = new Pose2d(15, -6, Math.toRadians(0));
        Pose2d nextTile = new Pose2d(52,-3,Math.toRadians(90));
        Pose2d spike3Avoid = new Pose2d(48, -12,Math.toRadians(180));
        Pose2d spike2Avoid = new Pose2d(48, -8, Math.toRadians(150));
        Pose2d spike1Avoid = new Pose2d(30.5, -6, Math.toRadians(90));
        Pose2d MiddleTile = new Pose2d(52,70, Math.toRadians(90));
        Pose2d spike3 = new Pose2d(39, -12, Math.toRadians(180));
        Pose2d spike2 = new Pose2d(43.5, -2, Math.toRadians(180));
        Pose2d spike1 = new Pose2d(30.5, 3.5, Math.toRadians(90));
        Pose2d boardRight = new Pose2d(31.5, 72.3, Math.toRadians(90));
        Pose2d boardMiddle = new Pose2d(23.5, 72.3, Math.toRadians(90));
        Pose2d boardLeft = new Pose2d(14, 72.3, Math.toRadians(90));
        Pose2d park = new Pose2d(52, 86, Math.toRadians(90));

        TrajectorySequence linetoFirstTile = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
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
        TrajectorySequence avoid1 = drive.trajectorySequenceBuilder(spike1)
                .lineToLinearHeading(spike1Avoid)
                .build();
        TrajectorySequence avoid2 = drive.trajectorySequenceBuilder(spike2)
                .lineToLinearHeading(spike2Avoid)
                .build();
        TrajectorySequence avoid3 = drive.trajectorySequenceBuilder(spike3)
                .lineToLinearHeading(spike3Avoid)
                .build();
        TrajectorySequence toNextLeft = drive.trajectorySequenceBuilder(spike1Avoid)
                .lineToLinearHeading(nextTile)
                .build();
        TrajectorySequence toNextCenter = drive.trajectorySequenceBuilder(spike2Avoid)
                .lineToLinearHeading(nextTile)
                .build();
        TrajectorySequence toNextRight = drive.trajectorySequenceBuilder(spike3Avoid)
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

            if(gamepad2.b) {
                elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            left_lift.setPower(-gamepad2.right_stick_y);
            right_lift.setPower(-gamepad2.right_stick_y);
            elbow.setPower(gamepad2.left_stick_y);
            wrist.setPosition(0.63);

            telemetry.addData("Left Lift Encoder", left_lift.getCurrentPosition());
            telemetry.addData("Right Lift Encoder", right_lift.getCurrentPosition());
            telemetry.addData("Elbow Encoder", elbow.getCurrentPosition());
            telemetry.addData("Location", propPipeline.getLocation());
            telemetry.update();
        }

        waitForStart();

        if (!isStopRequested()) {

            Location location = propPipeline.getLocation();

            wrist.setPosition(0.24);
            door.setPosition(0.75);
            arm1.ArmToPos(-2000, 0.5);
            arm2.ArmToPos(160, 1);

            if (location == PropPipeline.Location.LEFT) {
                // Movements for left spot

                telemetry.addData("Position", "Left");
                telemetry.update();

                drive.followTrajectorySequence(linetoFirstTile);
                drive.followTrajectorySequence(toSpike1);
                sleep(500);
                door.setPosition(0.1);
                sleep(200);
                arm2.ArmToPos(0,1);
                sleep(500);
                door.setPosition(0.95);
                drive.followTrajectorySequence(avoid1);
                drive.followTrajectorySequence(toNextLeft);
                arm2.ArmToPos(200,1);
                wrist.setPosition(0.3);
                drive.followTrajectorySequence(toMiddle);
                drive.followTrajectorySequence(toBoardLeft);
                arm1.ArmToPos(-664, 0.5);
                wrist.setPosition(0.6);
                arm2.ArmToPos(-812, 0.65);
                intake.setPower(-1);
                sleep(500);
                intake.setPower(0);
                sleep(1000);
                door.setPosition(0.1);
                sleep(749);
                arm2.ArmToPos(-900, 0.5);
                sleep(500);
                door.setPosition(0.95);
                sleep(1000);
                arm1.ArmToPos(-1850, 0.5);
                drive.followTrajectorySequence(leftPark);
                arm2.ArmToPos(0,1);

            }
            if (location == PropPipeline.Location.CENTER) {
                // Movements for center spot
                telemetry.addData("Position", "Center");
                telemetry.update();

                drive.followTrajectorySequence(linetoFirstTile);
                drive.followTrajectorySequence(toSpike2);
                sleep(500);
                door.setPosition(0.1);
                sleep(200);
                arm2.ArmToPos(0,1);
                sleep(500);
                door.setPosition(0.95);
                drive.followTrajectorySequence(avoid2);
                drive.followTrajectorySequence(toNextCenter);
                arm2.ArmToPos(200,1);
                wrist.setPosition(0.3);
                drive.followTrajectorySequence(toMiddle);
                drive.followTrajectorySequence(toBoardCenter);
                arm1.ArmToPos(-664, 0.5);
                wrist.setPosition(0.6);
                arm2.ArmToPos(-812, 0.65);
                intake.setPower(-1);
                sleep(500);
                intake.setPower(0);
                sleep(1000);
                door.setPosition(0.1);
                sleep(749);
                arm2.ArmToPos(-900, 0.5);
                sleep(500);
                door.setPosition(0.95);
                sleep(1000);
                arm1.ArmToPos(-1850, 0.5);
                drive.followTrajectorySequence(centerPark);
                arm2.ArmToPos(0,1);

            }
            if (location == PropPipeline.Location.RIGHT) {
                // Movements for Right spot

                telemetry.addData("Position", "Right");
                telemetry.update();

                drive.followTrajectorySequence(linetoFirstTile);
                drive.followTrajectorySequence(toSpike3);
                sleep(500);
                door.setPosition(0.1);
                sleep(200);
                arm2.ArmToPos(0,1);
                sleep(500);
                door.setPosition(0.95);
                drive.followTrajectorySequence(avoid3);
                drive.followTrajectorySequence(toNextRight);
                arm2.ArmToPos(200,1);
                wrist.setPosition(0.3);
                drive.followTrajectorySequence(toMiddle);
                drive.followTrajectorySequence(toBoardRight);
                arm1.ArmToPos(-664, 0.5);
                wrist.setPosition(0.6);
                arm2.ArmToPos(-812, 0.65);
                intake.setPower(-1);
                sleep(500);
                intake.setPower(0);
                sleep(1000);
                door.setPosition(0.1);
                sleep(749);
                arm2.ArmToPos(-900, 0.5);
                sleep(500);
                door.setPosition(0.95);
                sleep(1000);
                arm1.ArmToPos(-1850, 0.5);
                drive.followTrajectorySequence(rightPark);
                arm2.ArmToPos(0,1);

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