package org.firstinspires.ftc.teamcode.auto;

        import static org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection.Location.CENTER;
        import static org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection.Location.LEFT;

        import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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

        CenterStageDetection detector = new CenterStageDetection();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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
        camera.startStreaming(width, height, OpenCvCameraRotation.UPSIDE_DOWN);

        CenterStageDetection.ColorDetected colorLeft = detector.getColorLeft();
        CenterStageDetection.ColorDetected colorMiddle = detector.getColorMiddle();

        telemetry.addData("Detecting Left: ", colorLeft);
        telemetry.addData("Detecting Center: ", colorMiddle);
        telemetry.update();

        waitForStart();

        if (!isStopRequested()) {

            colorLeft = detector.getColorLeft();
            colorMiddle = detector.getColorMiddle();

            telemetry.addData("Detecting Left: ", colorLeft);
            telemetry.addData("Detecting Center: ", colorMiddle);
            telemetry.addData("Location", detector.getLocation());
//
            telemetry.update();

            TrajectorySequence ToBackdropLeft = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90.00)))
                    .waitSeconds(2)
                    .splineToSplineHeading(new Pose2d(-36, 20, Math.toRadians(180.00)), Math.toRadians(180.00))
                    .build();
            drive.setPoseEstimate(ToBackdropLeft.start());

            TrajectorySequence ToBackdropCenter = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90.00)))
                    .waitSeconds(2)
                    .splineToSplineHeading(new Pose2d(-36, 25, Math.toRadians(180.00)), Math.toRadians(180.00))
                    .build();
            drive.setPoseEstimate(ToBackdropCenter.start());

            TrajectorySequence ToBackdropRight = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90.00)))
                    .waitSeconds(2)
                    .splineToSplineHeading(new Pose2d(-36, 30, Math.toRadians(180.00)), Math.toRadians(180.00))
                    .build();
            drive.setPoseEstimate(ToBackdropRight.start());


            if (detector.getLocation()==LEFT) {
                // Movements for left spot
                telemetry.addData("Position", "Left");
                telemetry.update();
                drive.followTrajectorySequence(ToBackdropLeft);

            } else if (detector.getLocation()==CENTER) {
                // Movements for center spot
                telemetry.addData("Position", "CENTER");
                telemetry.update();
                drive.followTrajectorySequence(ToBackdropCenter);

            } else {
                // Movements for right spot
                telemetry.addData("Position", "RIGHT");
                telemetry.update();
                drive.followTrajectorySequence(ToBackdropRight);

            }

            camera.stopStreaming();
        }
    }
}
