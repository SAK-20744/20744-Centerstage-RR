package org.firstinspires.ftc.teamcode.opModes.noobRobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.vision.Pipething;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@Config
@Autonomous(name= "Video-Test" , group = "advanced")
public class VideoV1 extends LinearOpMode {
    private Pipething propPipeline;
    private VisionPortal portal;


    public void runOpMode() throws InterruptedException {

        propPipeline = new Pipething();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(propPipeline)
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();
        FtcDashboard.getInstance().startCameraStream(propPipeline, 60);

        waitForStart();

        if (isStopRequested()) return;
    }
}