package org.firstinspires.ftc.teamcode.auto.testing;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.myPropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Disabled
@Autonomous(name = "PipelineTest")
public class PipelineTest extends LinearOpMode {

    private myPropPipeline propPipeline;
    private VisionPortal portal;

    @Override
    public void runOpMode() throws InterruptedException {

        propPipeline = new myPropPipeline();
        portal = new VisionPortal.Builder()
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