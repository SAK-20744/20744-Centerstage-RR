package org.firstinspires.ftc.teamcode.opModes.auto.testing;

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
import org.firstinspires.ftc.teamcode.subsystems.vision.Pipeline;
import org.firstinspires.ftc.teamcode.subsystems.vision.PropPipeline;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.subsystems.vision.trc.FtcEocvColorBlobProcessor;
import org.firstinspires.ftc.teamcode.subsystems.vision.trc.FtcVisionEocvColorBlob;
import org.firstinspires.ftc.teamcode.subsystems.vision.trc.RobotParams;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//@Disabled
@Autonomous(name = "TCRPipelineTest")
public class TRCPipelineTest extends LinearOpMode {

    private Pipeline pipeline;
    private VisionPortal portal;
    private AprilTagProcessor aprilTag;

    public FtcVisionEocvColorBlob purplePixelVision;
    private FtcEocvColorBlobProcessor purplePixelProcessor;
    public FtcVisionEocvColorBlob greenPixelVision;
    private FtcEocvColorBlobProcessor greenPixelProcessor;
    public FtcVisionEocvColorBlob yellowPixelVision;
    private FtcEocvColorBlobProcessor yellowPixelProcessor;
    public FtcVisionEocvColorBlob whitePixelVision;
    private FtcEocvColorBlobProcessor whitePixelProcessor;
    public FtcVisionEocvColorBlob redBlobVision;
    private FtcEocvColorBlobProcessor redBlobProcessor;
    public FtcVisionEocvColorBlob blueBlobVision;
    private FtcEocvColorBlobProcessor blueBlobProcessor;

    @Override
    public void runOpMode() throws InterruptedException {

        pipeline = new Pipeline();
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(3);

        purplePixelVision = new FtcVisionEocvColorBlob(
                "PurplePixel", colorConversion, purplePixelColorThresholds, pixelFilterContourParams, true,
                RobotParams.cameraRect, RobotParams.worldRect, true);
        purplePixelProcessor = purplePixelVision.getVisionProcessor();
//        visionProcessorsList.add(purplePixelProcessor);

        greenPixelVision = new FtcVisionEocvColorBlob(
                "GreenPixel", colorConversion, greenPixelColorThresholds, pixelFilterContourParams, true,
                RobotParams.cameraRect, RobotParams.worldRect, true);
        greenPixelProcessor = greenPixelVision.getVisionProcessor();
//        visionProcessorsList.add(greenPixelProcessor);

        yellowPixelVision = new FtcVisionEocvColorBlob(
                "YellowPixel", colorConversion, yellowPixelColorThresholds, pixelFilterContourParams, true,
                RobotParams.cameraRect, RobotParams.worldRect, true);
        yellowPixelProcessor = yellowPixelVision.getVisionProcessor();
//        visionProcessorsList.add(yellowPixelProcessor);

        whitePixelVision = new FtcVisionEocvColorBlob(
                "WhitePixel", colorConversion, whitePixelColorThresholds, pixelFilterContourParams, true,
                RobotParams.cameraRect, RobotParams.worldRect, true);
        whitePixelProcessor = whitePixelVision.getVisionProcessor();
//        visionProcessorsList.add(whitePixelProcessor);

        redBlobVision = new FtcVisionEocvColorBlob(
                "RedBlob", colorConversion, redBlobColorThresholds, blobFilterContourParams, true,
                RobotParams.cameraRect, RobotParams.worldRect, true);
        redBlobProcessor = redBlobVision.getVisionProcessor();
//        visionProcessorsList.add(redBlobProcessor);

        blueBlobVision = new FtcVisionEocvColorBlob(
                "BlueBlob", colorConversion, blueBlobColorThresholds, blobFilterContourParams, true,
                RobotParams.cameraRect, RobotParams.worldRect, true);
        blueBlobProcessor = blueBlobVision.getVisionProcessor();
//        visionProcessorsList.add(blueBlobProcessor);

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
//            telemetry.addData("Location", propPipeline.getLocation());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
//            telemetry.addData("Location", propPipeline.getLocation());
            telemetry.update();
        }
    }
}