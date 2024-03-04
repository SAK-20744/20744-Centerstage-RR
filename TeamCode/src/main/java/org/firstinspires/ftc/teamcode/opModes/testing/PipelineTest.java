package org.firstinspires.ftc.teamcode.opModes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.vision.old.PropPipeline;
import org.firstinspires.ftc.teamcode.subsystems.vision.old.SlotPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

//@Disabled
@Autonomous(name = "PipelineTest")
public class PipelineTest extends LinearOpMode {

    private SlotPipeline propPipeline;
    private VisionPortal myPortal;

    @Override
    public void runOpMode() throws InterruptedException {

        propPipeline = new SlotPipeline();
        myPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .setCameraResolution(new Size(720, 1280))
                .addProcessor(propPipeline)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(propPipeline, 30);

        while (opModeInInit()) {
            telemetry.addData("Location", propPipeline.getLocation());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Location", propPipeline.getLocation());
            telemetry.update();


        }
    }
}