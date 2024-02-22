package org.firstinspires.ftc.teamcode.subsystems.vision.old.owen;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class calibrateVision extends LinearOpMode {

    @Override
    public void runOpMode() {
        callReadPipeline crp = new callReadPipeline(hardwareMap);
        crp.StartPipeline();
        while(!opModeIsActive()) {
            telemetry.addLine("Initialization Complete!");
            telemetry.update();
            if (isStopRequested()) {
                crp.StopPipeline();break;
            }
        }
        waitForStart();
        while (opModeIsActive()) {
            if (isStopRequested()) {
                crp.StopPipeline();break;
            }
            telemetry.addLine("\n   Values:");
            telemetry.addData("H Value", crp.getH());
            telemetry.addData("S Value", crp.getS());
            telemetry.addData("V Value", crp.getV());
            telemetry.update();
        }
    }
}