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

        import org.checkerframework.checker.units.qual.A;
        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
        import org.firstinspires.ftc.teamcode.subsystems.Arm1;
        import org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection;
        import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
        import org.openftc.easyopencv.OpenCvCamera;
        import org.openftc.easyopencv.OpenCvCameraFactory;
        import org.openftc.easyopencv.OpenCvCameraRotation;
        import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class AutoCenterStage2 extends LinearOpMode {

    @Override
    public void runOpMode() {

        CenterStageDetection detector;
        SampleMecanumDrive drive;
        Servo wrist;
        DcMotor left_lift;
        DcMotor right_lift;
        CRServo leftArm;
        CRServo rightArm;
        CRServo intake;
        Servo door;
        AnalogInput left;
        AnalogInput right;
        Arm1 arm1 = new Arm1(hardwareMap);

        boolean ButtonXBlock = false;
        double wristservoposition = 0.63;

        boolean isLeft = false;
        boolean isMiddle = false;
        boolean isRight = false;

        int width = 320;
        int height = 240;

        detector = new CenterStageDetection();
        drive = new SampleMecanumDrive(hardwareMap);
        wrist = hardwareMap.get(Servo.class, "wrist");
        left_lift = hardwareMap.get(DcMotor.class, "left_lift");
        right_lift = hardwareMap.get(DcMotor.class, "right_lift");
        leftArm = hardwareMap.get(CRServo.class, "leftArm");
        rightArm = hardwareMap.get(CRServo.class, "rightArm");
        intake = hardwareMap.get(CRServo.class, "intake");
        door = hardwareMap.get(Servo.class, "door");
        left = hardwareMap.get(AnalogInput.class, "left");
        right = hardwareMap.get(AnalogInput.class, "right");

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
            arm1.ArmToPos(-1000, 0.3);
            leftArm.setPower(0);
            rightArm.setPower(0);


            colorLeft = detector.getColorLeft();
            colorMiddle = detector.getColorMiddle();

            telemetry.addData("Detecting Left: ", colorLeft);
            telemetry.addData("Detecting Center: ", colorMiddle);
            telemetry.addData("Location", detector.getLocation());
//
            telemetry.update();

            TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90.00)))
                    .waitSeconds(2)
                    .splineToSplineHeading(new Pose2d(-3, 25, Math.toRadians(90)), Math.toRadians(105))
                    .build();
            drive.setPoseEstimate(leftPurple.start());

            TrajectorySequence centerPurple = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90.00)))
                    .waitSeconds(2)
                    .splineToSplineHeading(new Pose2d(4, 30, Math.toRadians(90)), Math.toRadians(90))
                    .build();
            drive.setPoseEstimate(leftPurple.start());

            TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90.00)))
                    .waitSeconds(2)
                    .splineToSplineHeading(new Pose2d(10, 25, Math.toRadians(90)), Math.toRadians(75))
                    .build();
            drive.setPoseEstimate(leftPurple.start());

            if (detector.getLocation()==LEFT) {
                // Movements for left spot
                telemetry.addData("Position", "Left");
                telemetry.update();
                drive.followTrajectorySequence(leftPurple);

            } else if (detector.getLocation()==CENTER) {
                // Movements for center spot
                telemetry.addData("Position", "CENTER");
                telemetry.update();
                drive.followTrajectorySequence(centerPurple);

            } else {
                // Movements for right spot
                telemetry.addData("Position", "RIGHT");
                telemetry.update();
                drive.followTrajectorySequence(rightPurple);

            }

            camera.stopStreaming();
        }
    }
}