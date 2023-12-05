package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm1;
import org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection;
import org.firstinspires.ftc.teamcode.subsystems.ServoArm;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Disabled
@Autonomous
public class AutoCenterStage extends LinearOpMode {

    @Override
    public void runOpMode() {

        CenterStageDetection detector = new CenterStageDetection();
        OpenCvCamera camera;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        Servo door = hardwareMap.get(Servo.class, "door");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");

        Arm1 arm1 = (new Arm1(hardwareMap));
        ServoArm arm2 = new ServoArm(hardwareMap);

        boolean isLeft = false;
        boolean isMiddle = false;
        boolean isRight = false;

        int width = 320;
        int height = 240;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        camera.openCameraDevice();
        camera.setPipeline(detector);
        camera.startStreaming(width, height, OpenCvCameraRotation.SIDEWAYS_LEFT);
        CenterStageDetection.ColorDetected colorLeft = detector.getColorLeft();
        CenterStageDetection.ColorDetected colorMiddle = detector.getColorMiddle();

        telemetry.addData("Detecting Left: ", colorLeft);
        telemetry.addData("Detecting Center: ", colorMiddle);
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {

            colorLeft = detector.getColorLeft();
            colorMiddle = detector.getColorMiddle();

            if ((colorLeft == CenterStageDetection.ColorDetected.CYAN) || (colorLeft == CenterStageDetection.ColorDetected.MAGENTA))
                isLeft = true;
            else if ((colorMiddle == CenterStageDetection.ColorDetected.CYAN) || (colorMiddle == CenterStageDetection.ColorDetected.MAGENTA))
                isMiddle = true;
            else
                isRight = true;

            TrajectorySequence ToBackdropLeft = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90.00)))
                    .splineToSplineHeading(new Pose2d(-36, 23, Math.toRadians(180.00)), Math.toRadians(180.00))
                    .build();
            drive.setPoseEstimate(ToBackdropLeft.start());

            TrajectorySequence ToBackdropCenter = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90.00)))
                    .splineToSplineHeading(new Pose2d(-36, 25.5, Math.toRadians(180.00)), Math.toRadians(180.00))
                    .build();
            drive.setPoseEstimate(ToBackdropCenter.start());

            TrajectorySequence ToBackdropRight = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90.00)))
                    .splineToSplineHeading(new Pose2d(-36, 28, Math.toRadians(180.00)), Math.toRadians(180.00))
                    .build();
            drive.setPoseEstimate(ToBackdropRight.start());


            telemetry.addData("Detecting Left: ", colorLeft);
            telemetry.addData("Detecting Center: ", colorMiddle);
//
            telemetry.update();

            if (isLeft) {
                // Movements for left spot
                telemetry.addData("Position", "Left");
                telemetry.update();
                drive.followTrajectorySequence(ToBackdropCenter);
                arm1.ArmToDeg(90, 0.5);
                arm2.runToProfile(45);
                wrist.setPosition(0.63);

            } else if (isMiddle) {
                // Movements for center spot
                telemetry.addData("Position", "Right");
                telemetry.update();
                drive.followTrajectorySequence(ToBackdropCenter);
                arm1.ArmToDeg(90, 0.5);
                arm2.runToProfile(45);
                wrist.setPosition(0.63);

            } else {
                // Movements for right spot
                telemetry.addData("Position", "Middle");
                telemetry.update();
                drive.followTrajectorySequence(ToBackdropCenter);
                arm1.ArmToDeg(90, 0.5);
                arm2.runToProfile(45);
                wrist.setPosition(0.63);

            }

            camera.stopStreaming();
        }
    }
}


