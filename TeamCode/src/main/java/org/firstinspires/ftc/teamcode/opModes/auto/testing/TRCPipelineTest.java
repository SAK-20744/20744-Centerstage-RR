package org.firstinspires.ftc.teamcode.opModes.auto.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.vision.Pipeline;
import org.firstinspires.ftc.teamcode.subsystems.vision.PropPipeline;
import org.firstinspires.ftc.teamcode.subsystems.vision.trc.FtcEocvColorBlobProcessor;
import org.firstinspires.ftc.teamcode.subsystems.vision.trc.FtcVisionEocvColorBlob;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//@Disabled
@Autonomous(name = "PipelineTest")
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