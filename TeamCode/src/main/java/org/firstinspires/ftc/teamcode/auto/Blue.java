package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection.Location.CENTER;
import static org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection.Location.LEFT;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm1;
import org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection;
import org.firstinspires.ftc.teamcode.subsystems.ServoArm;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

//@Disabled
@Autonomous
public class Blue extends LinearOpMode {

    private CenterStageDetection detector = new CenterStageDetection();
    private SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    private Arm1 arm1 = (new Arm1(hardwareMap));
    private ServoArm arm2 = new ServoArm(hardwareMap);

    private Servo wrist = hardwareMap.get(Servo.class, "wrist");
    private DcMotor left_lift = hardwareMap.get(DcMotor.class, "left_lift");
    private DcMotor right_lift = hardwareMap.get(DcMotor.class, "right_lift");
    private CRServo leftArm = hardwareMap.get(CRServo.class, "leftArm");
    private CRServo rightArm = hardwareMap.get(CRServo.class, "rightArm");
    private CRServo intake = hardwareMap.get(CRServo.class, "intake");
    private Servo door = hardwareMap.get(Servo.class, "door");

    private boolean ButtonXBlock = false;
    private double wristservoposition = 0.63;

    private boolean isLeft = false;
    private boolean isMiddle = false;
    private boolean isRight = false;

    CenterStageDetection.Location location;


    @Override
    public void runOpMode() throws InterruptedException{

        int width = 320;
        int height = 240;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.openCameraDevice();
        camera.setPipeline(detector);
        camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);

        CenterStageDetection.ColorDetected colorLeft = detector.getColorLeft();
        CenterStageDetection.ColorDetected colorMiddle = detector.getColorMiddle();

        telemetry.addData("Detecting Left: ", colorLeft);
        telemetry.addData("Detecting Center: ", colorMiddle);
        telemetry.update();

        Pose2d startPose = new Pose2d(0,0,Math.toRadians(90));

        Trajectory toBackdropLeft = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-22,19, Math.toRadians(180)))
                .build();
//            drive.setPoseEstimate((new Pose2d(0,0,Math.toRadians(90))));

        Trajectory toBackdropCenter = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-22,25, Math.toRadians(180)))
                .build();
//            drive.setPoseEstimate((new Pose2d(0,0,Math.toRadians(90))));

        Trajectory toBackdropRight = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-22,35, Math.toRadians(180)))
                .build();
//            drive.setPoseEstimate((new Pose2d(0,0,Math.toRadians(90))));


        Trajectory leftStrafe = drive.trajectoryBuilder(toBackdropLeft.end())
                .lineToSplineHeading(new Pose2d(-22,0, Math.toRadians(180)))
                .build();
//            drive.setPoseEstimate((new Pose2d(0,0,Math.toRadians(90))));

        Trajectory centerStrafe = drive.trajectoryBuilder(toBackdropCenter.end())
                .lineToSplineHeading(new Pose2d(-22,0, Math.toRadians(180)))
                .build();
//            drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(90)));

        Trajectory rightStrafe = drive.trajectoryBuilder(toBackdropRight.end())
                .lineToSplineHeading(new Pose2d(-22,0, Math.toRadians(180)))
                .build();
//            drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(90)));

        Trajectory park = drive.trajectoryBuilder(rightStrafe.end())
                .lineToSplineHeading(new Pose2d(-36,0, Math.toRadians(180)))
                .build();
//            drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(180)));

        // move motors to -660 for init
        wrist.setPosition(wristservoposition);
        door.setPosition(0);

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

            leftArm.setPower(0.45);
            rightArm.setPower(-0.45);
            sleep(300);
            leftArm.setPower(0);
            rightArm.setPower(0);

            wrist.setPosition(0.63);
            door.setPosition(0.75);

            arm2.runToProfile(0);

            sleep(2000);

            colorLeft = detector.getColorLeft();
            colorMiddle = detector.getColorMiddle();

            location = detector.getLocation();

            telemetry.addData("Detecting Left: ", colorLeft);
            telemetry.addData("Detecting Center: ", colorMiddle);
            telemetry.addData("Location", location);
//
            telemetry.update();


            if (location == LEFT) {
                // Movements for left spot
                // Movements for right spot
                telemetry.addData("Position", "LEFT");
                telemetry.update();

                drive.followTrajectory(toBackdropLeft);

                arm2.runToProfile(0);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "LEFT");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

                arm1.ArmToPos(-500, 0.5);

                wrist.setPosition(0.7);

                arm2.runToProfile(95);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "LEFT");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

                sleep(750);
                door.setPosition(0);
                sleep(1000);

                arm2.runToProfile(120);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "LEFT");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

                door.setPosition(0.75);

                arm1.ArmToPos(-1000, 1);

                arm2.runToProfile(0);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "LEFT");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();

                }

//                arm1.ArmToPos(0,0.5);
//
//                arm2.runToProfile(0);
//                while( (arm2.isBusy()) && !isStopRequested()) {
//                    arm2.updateServoArm();
//                    telemetry.addData("Position", "RIGHT");
//                    telemetry.addData("Arm2" , arm2.getLocation());
//                    telemetry.addData("Arm2 State" , arm2.isBusy());
//                    telemetry.update();
//                }

                drive.followTrajectory(leftStrafe);


            } else if (location == CENTER) {
                // Movements for center spot
                // Movements for right spot
                telemetry.addData("Position", "CENTER");
                telemetry.update();

                drive.followTrajectory(toBackdropCenter);

                arm2.runToProfile(0);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "CENTER");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

                arm1.ArmToPos(-500, 0.5);

                wrist.setPosition(0.7);

                arm2.runToProfile(95);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "CENTER");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

                sleep(750);
                door.setPosition(0);
                sleep(1000);

                arm2.runToProfile(120);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "CENTER");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

                door.setPosition(0.75);

                arm1.ArmToPos(-1000, 1);

                arm2.runToProfile(0);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "CENTER");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

//                arm1.ArmToPos(0,0.5);
//
//                arm2.runToProfile(0);
//                while( (arm2.isBusy()) && !isStopRequested()) {
//                    arm2.updateServoArm();
//                    telemetry.addData("Position", "RIGHT");
//                    telemetry.addData("Arm2" , arm2.getLocation());
//                    telemetry.addData("Arm2 State" , arm2.isBusy());
//                    telemetry.update();
//                }

                drive.followTrajectory(centerStrafe);


            } else {
                // Movements for right spot
                telemetry.addData("Position", "RIGHT");
                telemetry.update();

                drive.followTrajectory(toBackdropRight);

                arm2.runToProfile(0);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "RIGHT");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

                arm1.ArmToPos(-500, 0.5);

                wrist.setPosition(0.7);

                arm2.runToProfile(95);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "RIGHT");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

                sleep(750);
                door.setPosition(0);
                sleep(1000);

                arm2.runToProfile(120);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "RIGHT");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

                door.setPosition(0.75);

                arm1.ArmToPos(-1000, 1);

                arm2.runToProfile(0);
                while( (arm2.isBusy()) && !isStopRequested()) {
                    arm2.updateServoArm();
                    telemetry.addData("Position", "RIGHT");
                    telemetry.addData("Arm2" , arm2.getLocation());
                    telemetry.addData("Arm2 State" , arm2.isBusy());
                    telemetry.update();
                }

//                arm1.ArmToPos(0,0.5);
//
//                arm2.runToProfile(0);
//                while( (arm2.isBusy()) && !isStopRequested()) {
//                    arm2.updateServoArm();
//                    telemetry.addData("Position", "RIGHT");
//                    telemetry.addData("Arm2" , arm2.getLocation());
//                    telemetry.addData("Arm2 State" , arm2.isBusy());
//                    telemetry.update();
//                }


                drive.followTrajectory(rightStrafe);

            }

            drive.followTrajectory(park);

            sleep(30000);

            camera.stopStreaming();

        }
    }
}