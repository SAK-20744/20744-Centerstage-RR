package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection.Location.CENTER;
import static org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection.Location.LEFT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm1;
import org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection;
import org.firstinspires.ftc.teamcode.subsystems.ServoArm;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class ArmAutoV1 extends LinearOpMode {

    @Override
    public void runOpMode() {

        CenterStageDetection detector = new CenterStageDetection();
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

        boolean ButtonXBlock = false;
        double wristservoposition = 0.63;

        boolean isLeft = false;
        boolean isMiddle = false;
        boolean isRight = false;

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

        // move motors to -660 for init
        wrist.setPosition(wristservoposition);

        while (!(opModeIsActive() || isStopRequested())) {
            left_lift.setPower(-gamepad2.right_stick_y);
            right_lift.setPower(-gamepad2.right_stick_y);
            leftArm.setPower(-gamepad2.left_stick_y);
            rightArm.setPower(gamepad2.left_stick_y);
            telemetry.addData("Left Lift Encoder", left_lift.getCurrentPosition());
            telemetry.addData("Right Lift Encoder", left_lift.getCurrentPosition());
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
            //arm1.ArmToPos(-500, 0.3);
            leftArm.setPower(0);
            rightArm.setPower(0);

            wrist.setPosition(0.63);


            colorLeft = detector.getColorLeft();
            colorMiddle = detector.getColorMiddle();

            telemetry.addData("Detecting Left: ", colorLeft);
            telemetry.addData("Detecting Center: ", colorMiddle);
            telemetry.addData("Location", detector.getLocation());
//
            telemetry.update();

            TrajectorySequence ToBackdropLeft = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90.00)))
                    .splineToSplineHeading(new Pose2d(-32, 19, Math.toRadians(180.00)), Math.toRadians(180.00))
                    .build();
            drive.setPoseEstimate(ToBackdropLeft.start());

            TrajectorySequence ToBackdropCenter = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90.00)))
                    .splineToSplineHeading(new Pose2d(-32, 22, Math.toRadians(180.00)), Math.toRadians(180.00))
                    .build();
            drive.setPoseEstimate(ToBackdropCenter.start());

            TrajectorySequence ToBackdropRight = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90.00)))
                    .splineToSplineHeading(new Pose2d(-32, 25, Math.toRadians(180.00)), Math.toRadians(180.00))
                    .build();
            drive.setPoseEstimate(ToBackdropRight.start());

            if (isLeft) {
                // Movements for left spot
                telemetry.addData("Position", "Left");
                telemetry.update();
                drive.followTrajectorySequence(ToBackdropLeft);
                arm1.ArmToPos(-1000, 0.5);
                arm2.runToProfile(80);
                wrist.setPosition(0.63);
                door.setPosition(0.9);
                sleep(3000);

            } else if (isMiddle) {
                // Movements for center spot
                telemetry.addData("Position", "Right");
                telemetry.update();
                drive.followTrajectorySequence(ToBackdropCenter);
                arm2.runToProfile(60);
                arm1.ArmToPos(-1000, 0.5);

//                sleep(1000);
//                wrist.setPosition(0.63);
//                door.setPosition(0.9);
//                sleep(3000);

            } else {
                // Movements for right spot
                telemetry.addData("Position", "Middle");
                telemetry.update();
                drive.followTrajectorySequence(ToBackdropRight);
                arm1.ArmToPos(-1000, 0.5);
                arm2.runToProfile(80);
                wrist.setPosition(0.63);
                door.setPosition(0.9);
                sleep(3000);

            }

            camera.stopStreaming();

        }
    }
}