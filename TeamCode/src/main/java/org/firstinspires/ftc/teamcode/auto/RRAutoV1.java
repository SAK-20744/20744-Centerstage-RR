package org.firstinspires.ftc.teamcode.auto;

import android.graphics.Bitmap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.PipelineRecordingParameters;

@Autonomous
public class RRAutoV1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        OpenCvCamera camera = new OpenCvCamera(hardwareMap.get(OpenCvCamera, "Webcam 1"));

        TrajectorySequence ToBackdropCenter = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90.00)))
                .splineToSplineHeading(new Pose2d(-36, 25.5, Math.toRadians(180.00)), Math.toRadians(180.00))
                .build();
        drive.setPoseEstimate(ToBackdropCenter.start());

//        TrajectorySequence toStack = drive.trajectorySequenceBuilder(ToBackdropCenter.end())
//                .splineToSplineHeading(new Pose2d(48,24, Math.toRadians(180)), Math.toRadians(180))
//                .build();


        waitForStart();
        if (!isStopRequested()) {

            drive.followTrajectorySequence(ToBackdropCenter);

        }

    }
}
