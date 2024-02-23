/*package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm1;
import org.firstinspires.ftc.teamcode.subsystems.ServoArm;
import org.firstinspires.ftc.teamcode.subsystems.vision.callMainPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class VisionRedAuto extends LinearOpMode {

    callMainPipeline cp;
    int elementPos = 0;

    @Override
    public void runOpMode() {

        cp = new callMainPipeline(hardwareMap, "red");

//        CenterStageDetection detector = new CenterStageDetection();
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

        Pose2d MiddleTile = new Pose2d(15, -4, Math.toRadians(0));
        Pose2d spike3 = new Pose2d(28, -13, Math.toRadians(0));
        Pose2d spike2 = new Pose2d(31, -4, Math.toRadians(0));
        Pose2d spike1 = new Pose2d(30.5, 6, Math.toRadians(90));
        Pose2d boardRight = new Pose2d(18.5, -27, Math.toRadians(-90));
        Pose2d boardMiddle = new Pose2d(26, -27, Math.toRadians(-90));
        Pose2d boardLeft = new Pose2d(34.5, -27, Math.toRadians(-90));
        Pose2d park = new Pose2d(0, -34, Math.toRadians(-90));
        Pose2d start = drive.getPoseEstimate();
        Pose2d detect = new Pose2d(5.5,-1,Math.toRadians(0));


        TrajectorySequence toDetect = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(detect)
                .build();
        TrajectorySequence lineToMiddleTile = drive.trajectorySequenceBuilder(detect)
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

        boolean isLeft = false;
        boolean isMiddle = false;
        boolean isRight = false;

//        CenterStageDetection.Location location;
//
//        int width = 320;
//        int height = 240;
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
//        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//        camera.openCameraDevice();
//        camera.setPipeline(detector);
//        camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
//
//        CenterStageDetection.ColorDetected colorLeft = detector.getColorLeft();
//        CenterStageDetection.ColorDetected colorMiddle = detector.getColorMiddle();
//
//        telemetry.addData("Detecting Left: ", colorLeft);
//        telemetry.addData("Detecting Center: ", colorMiddle);
//        telemetry.update();

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
//            arm2.runToProfile(-5);
        }

        waitForStart();

        if (!isStopRequested()) {

            drive.followTrajectorySequence(toDetect);

            cp.StartPipeline();

            leftArm.setPower(0.45);
            rightArm.setPower(-0.45);
            sleep(450);
            leftArm.setPower(0);
            rightArm.setPower(0);

            wrist.setPosition(0.3);
            door.setPosition(0.75);
            sleep(1500);

            cp.StopPipeline();
            elementPos = cp.getElementPos();

            sleep(500);

//            colorLeft = detector.getColorLeft();
//            colorMiddle = detector.getColorMiddle();
//
//            location = detector.getLocation();
//
//            telemetry.addData("Detecting Left: ", colorLeft);
//            telemetry.addData("Detecting Center: ", colorMiddle);
//            telemetry.addData("Location", location);
//
            telemetry.update();

            arm1.ArmToPos(-1750, 1);

            if (elementPos == 1) {
                // Movements for left spot

                telemetry.addData("Position", "Left");
                telemetry.update();

                drive.followTrajectorySequence(lineToMiddleTile);

                sleep(1000);

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

                drive.followTrajectorySequence(toMiddleLeft);

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

                wrist.setPosition(0.55);

                arm2.runToProfile(95);
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

                arm2.runToProfile(120);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "Left");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

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
            if (elementPos == 2) {
                // Movements for center spot
                telemetry.addData("Position", "Center");
                telemetry.update();

                drive.followTrajectorySequence(lineToMiddleTile);

                sleep(1000);

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

                drive.followTrajectorySequence(toMiddleCenter);

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

                wrist.setPosition(0.55);

                arm2.runToProfile(95);
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

                arm2.runToProfile(120);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "Center");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

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
            if (elementPos == 3) {
                // Movements for Right spot

                telemetry.addData("Position", "Right");
                telemetry.update();

                drive.followTrajectorySequence(lineToMiddleTile);

                sleep(1000);

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

                drive.followTrajectorySequence(toMiddleRight);

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

                wrist.setPosition(0.55);

                arm2.runToProfile(95);
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

                arm2.runToProfile(120);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "Right");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

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

//            camera.stopStreaming();

        }


    }

}

 */