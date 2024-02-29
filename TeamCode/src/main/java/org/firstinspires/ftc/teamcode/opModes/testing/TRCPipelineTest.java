package org.firstinspires.ftc.teamcode.opModes.testing;

import static org.firstinspires.ftc.teamcode.subsystems.util.trc.RobotParams.*;
import static org.firstinspires.ftc.teamcode.subsystems.vision.Vision.blobFilterContourParams;
import static org.firstinspires.ftc.teamcode.subsystems.vision.Vision.blueBlobColorThresholds;
import static org.firstinspires.ftc.teamcode.subsystems.vision.Vision.colorConversion;
import static org.firstinspires.ftc.teamcode.subsystems.vision.Vision.greenPixelColorThresholds;
import static org.firstinspires.ftc.teamcode.subsystems.vision.Vision.pixelFilterContourParams;
import static org.firstinspires.ftc.teamcode.subsystems.vision.Vision.purplePixelColorThresholds;
import static org.firstinspires.ftc.teamcode.subsystems.vision.Vision.redBlobColorThresholds;
import static org.firstinspires.ftc.teamcode.subsystems.vision.Vision.whitePixelColorThresholds;
import static org.firstinspires.ftc.teamcode.subsystems.vision.Vision.yellowPixelColorThresholds;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.subsystems.util.trc.TrcOpenCvColorBlobPipeline;
import org.firstinspires.ftc.teamcode.subsystems.util.trc.TrcVisionTargetInfo;
import org.firstinspires.ftc.teamcode.subsystems.vision.Pipeline;
import org.firstinspires.ftc.teamcode.subsystems.util.trc.FtcEocvColorBlobProcessor;
import org.firstinspires.ftc.teamcode.subsystems.util.trc.FtcVisionEocvColorBlob;
import org.firstinspires.ftc.teamcode.subsystems.util.trc.RobotParams;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

//@Disabled
@Autonomous(name = "TCRPipelineTest")
public class TRCPipelineTest extends LinearOpMode {

    private Pipeline pipeline;
    private VisionPortal portal;
    private AprilTagProcessor aprilTag;
    private Vision myVision = new Vision();

    private int lastTeamPropPos = 0;

    private FtcVisionEocvColorBlob purplePixelVision;
    private FtcEocvColorBlobProcessor purplePixelProcessor;
    private FtcVisionEocvColorBlob greenPixelVision;
    private FtcEocvColorBlobProcessor greenPixelProcessor;
    private FtcVisionEocvColorBlob yellowPixelVision;
    private FtcEocvColorBlobProcessor yellowPixelProcessor;
    private FtcVisionEocvColorBlob whitePixelVision;
    private FtcEocvColorBlobProcessor whitePixelProcessor;
    private FtcVisionEocvColorBlob redBlobVision;
    private FtcEocvColorBlobProcessor redBlobProcessor;
    private FtcVisionEocvColorBlob blueBlobVision;
    private FtcEocvColorBlobProcessor blueBlobProcessor;

    @Override
    public void runOpMode() throws InterruptedException {

        pipeline = new Pipeline();

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);
        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        purplePixelVision = new FtcVisionEocvColorBlob(
                "PurplePixel", colorConversion, purplePixelColorThresholds, pixelFilterContourParams, true,
                cameraRect, worldRect, true);
        purplePixelProcessor = purplePixelVision.getVisionProcessor();
//        visionProcessorsList.add(purplePixelProcessor);

        greenPixelVision = new FtcVisionEocvColorBlob(
                "GreenPixel", colorConversion, greenPixelColorThresholds, pixelFilterContourParams, true,
                cameraRect, worldRect, true);
        greenPixelProcessor = greenPixelVision.getVisionProcessor();
//        visionProcessorsList.add(greenPixelProcessor);

        yellowPixelVision = new FtcVisionEocvColorBlob(
                "YellowPixel", colorConversion, yellowPixelColorThresholds, pixelFilterContourParams, true,
                cameraRect, worldRect, true);
        yellowPixelProcessor = yellowPixelVision.getVisionProcessor();
//        visionProcessorsList.add(yellowPixelProcessor);

        whitePixelVision = new FtcVisionEocvColorBlob(
                "WhitePixel", colorConversion, whitePixelColorThresholds, pixelFilterContourParams, true,
                cameraRect, worldRect, true);
        whitePixelProcessor = whitePixelVision.getVisionProcessor();
//        visionProcessorsList.add(whitePixelProcessor);

        redBlobVision = new FtcVisionEocvColorBlob(
                "RedBlob", colorConversion, redBlobColorThresholds, blobFilterContourParams, true,
                cameraRect, worldRect, true);
        redBlobProcessor = redBlobVision.getVisionProcessor();
//        visionProcessorsList.add(redBlobProcessor);

        blueBlobVision = new FtcVisionEocvColorBlob(
                "BlueBlob", colorConversion, blueBlobColorThresholds, blobFilterContourParams, true,
                cameraRect, worldRect, true);
        blueBlobProcessor = blueBlobVision.getVisionProcessor();
//        visionProcessorsList.add(blueBlobProcessor);


        FtcDashboard.getInstance().startCameraStream(pipeline, 30);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .setCameraResolution(new Size(720, 1280))
//                .addProcessor(propPipeline)
                .addProcessor(pipeline)
                .addProcessor(redBlobProcessor)
                .addProcessor(blueBlobProcessor)
                .addProcessor(aprilTag)
                .addProcessor(whitePixelProcessor)
                .addProcessor(greenPixelProcessor)
                .addProcessor(yellowPixelProcessor)
                .addProcessor(purplePixelProcessor)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(pipeline, 30);

        while (opModeInInit()) {
//            getDetectedTeamPropPosition(true);
//            getDetectedTeamPropPosition(true);
//            int location = getLastDetectedTeamPropPosition();
            telemetry.addData("Running", true);
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
//            telemetry.addData("Location", propPipeline.getLocation());
//
//            getDetectedTeamPropPosition(true);
//            int location = getLastDetectedTeamPropPosition();
            telemetry.addData("Running", true);
            telemetry.update();
        }
    }

//    private int getLastDetectedTeamPropPosition()
//    {
//        return lastTeamPropPos;
//    }   //getLastDetectedTeamPropPosition

//    private int getDetectedTeamPropPosition(boolean red)
//    {
//        int pos = 0;
//        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> teamPropInfo = null;
//
//        if (red)
//        {
//            if (redBlobVision != null)
//            {
//                teamPropInfo = redBlobVision.getBestDetectedTargetInfo(null, null, 0.0, 6);
//            }
//        }
//        else
//        {
//            if (blueBlobVision != null)
//            {
//                teamPropInfo = blueBlobVision.getBestDetectedTargetInfo(null, null, 0.0, 6);
//            }
//        }
//
//        if (teamPropInfo != null)
//        {
//            double teamPropXPos = teamPropInfo.rect.x + teamPropInfo.rect.width/2.0;
//            double oneHalfScreenWidth = CAM_IMAGE_WIDTH/2.0;
//
//            if (teamPropXPos < oneHalfScreenWidth && teamPropXPos > 0)
//            {
//                pos = 1;
//            }
//            else if (teamPropXPos > oneHalfScreenWidth && teamPropXPos < oneHalfScreenWidth*2)
//            {
//                pos = 2;
//            }
//            else
//            {
//                pos = 3;
//            }
//
//        }
//
//        if (pos != 0)
//        {
//            lastTeamPropPos = pos;
//        }
//
//        return pos;
//    }

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