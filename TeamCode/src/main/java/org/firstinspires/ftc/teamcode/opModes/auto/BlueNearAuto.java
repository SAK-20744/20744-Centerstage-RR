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
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.vision.PropPipeline;
import org.firstinspires.ftc.teamcode.subsystems.vision.PropPipeline.Location;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

//@Disabled
@Autonomous(name = "Blue Near 2+0")
public class BlueNearAuto extends LinearOpMode {

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

        Pose2d MiddleTile = new Pose2d(12, -2, Math.toRadians(0));
        Pose2d spike1 = new Pose2d(28, 11, Math.toRadians(0));
        Pose2d spike2 = new Pose2d(32, -4, Math.toRadians(0));
        Pose2d spike3 = new Pose2d(30.5, -6.75, Math.toRadians(-90));
        Pose2d boardLeft = new Pose2d(16.5, 24.7, Math.toRadians(90));
        Pose2d boardMiddle = new Pose2d(25, 24.7, Math.toRadians(90));
        Pose2d boardRight = new Pose2d(33.5, 24.7, Math.toRadians(90));
        Pose2d closePark = new Pose2d(0,32,Math.toRadians(90));
        Pose2d gatePark = new Pose2d(53.5,32,Math.toRadians(90));
        Pose2d park = closePark;

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

            if(gamepad2.dpad_up){
                park = gatePark;
            }
            if(gamepad2.dpad_down) {
                park = closePark;
            }

            if(park == gatePark)
                telemetry.addData("Park Position: Gate Side ", 0);
            else
                telemetry.addData("Park Position: Near Side ", 0);

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

        TrajectorySequence lineToMiddleTile = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(MiddleTile)
                .build();
        TrajectorySequence toSpike1 = drive.trajectorySequenceBuilder(MiddleTile)
                .lineToLinearHeading(spike1)
                .build();
        TrajectorySequence toSpike2 = drive.trajectorySequenceBuilder(MiddleTile)
                .lineToLinearHeading(spike2)
                .build();
        TrajectorySequence toSpike3 = drive.trajectorySequenceBuilder(MiddleTile)
                .lineToLinearHeading(spike3)
                .build();
        TrajectorySequence toMiddleLeft = drive.trajectorySequenceBuilder(spike1)
                .lineToLinearHeading(MiddleTile)
                .build();
        TrajectorySequence toMiddleCenter = drive.trajectorySequenceBuilder(spike2)
                .lineToLinearHeading(MiddleTile)
                .build();
        TrajectorySequence toMiddleRight = drive.trajectorySequenceBuilder(spike3)
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

        waitForStart();

        if (!isStopRequested()) {

            Location location = propPipeline.getLocation();

            wrist.setPosition(0.24);
            door.setPosition(0.75);
            arm1.ArmToPos(-2000, 0.5);
            arm2.ArmToPos(160, 1);

            if (location == LEFT) {
                // Movements for left spot
                telemetry.addData("Position", "Left");
                telemetry.update();

                drive.followTrajectorySequence(lineToMiddleTile);
                drive.followTrajectorySequence(toSpike1);
                sleep(500);
                door.setPosition(0.1);
                sleep(200);
                arm2.ArmToPos(0,1);
                sleep(500);
                door.setPosition(0.95);
                drive.followTrajectorySequence(toMiddleLeft);
                sleep(500);
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
            if (location == CENTER) {
                // Movements for center spot
                telemetry.addData("Position", "Center");
                telemetry.update();

                drive.followTrajectorySequence(lineToMiddleTile);
                drive.followTrajectorySequence(toSpike2);
                sleep(500);
                door.setPosition(0.1);
                sleep(200);
                arm2.ArmToPos(0,1);
                sleep(500);
                door.setPosition(0.95);
                drive.followTrajectorySequence(toMiddleCenter);
                sleep(500);
                drive.followTrajectorySequence(toBoardCenter);
                arm1.ArmToPos(-664, 0.5);
                wrist.setPosition(0.6);
                arm2.ArmToPos(-812, 65);
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
            if (location == RIGHT) {
                // Movements for right spot
                telemetry.addData("Position", "Right");
                telemetry.update();

                drive.followTrajectorySequence(lineToMiddleTile);
                drive.followTrajectorySequence(toSpike3);
                sleep(500);
                door.setPosition(0.1);
                sleep(200);
                arm2.ArmToPos(0,1);
                sleep(500);
                door.setPosition(0.95);
                drive.followTrajectorySequence(toMiddleRight);
                sleep(500);
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