package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//import org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection;
import org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class AutoCenterStage extends LinearOpMode {

    @Override
    public void runOpMode() {

        CenterStageDetection detector = new CenterStageDetection();
        OpenCvCamera camera;

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


            telemetry.addData("Detecting Left: ", colorLeft);
            telemetry.addData("Detecting Center: ", colorMiddle);
//
            telemetry.update();
        }

//        if (isLeft) {
//            // Movements for left spot
//            telemetry.addData("Position", "Left");
//            telemetry.update();
//            sleep(20000);
//        }
//        else if (isMiddle) {
//            // Movements for center spot
//            telemetry.addData("Position", "Right");
//            telemetry.update();
//            sleep(20000);
//        }
//        else {
//            // Movements for right spot
//            telemetry.addData("Position", "Middle");
//            telemetry.update();
//            sleep(20000);
//        }

        camera.stopStreaming();
    }
}
