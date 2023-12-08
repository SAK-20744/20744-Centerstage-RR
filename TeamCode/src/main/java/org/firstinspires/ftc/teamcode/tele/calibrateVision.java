package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.vision.OldPipeline;
import org.firstinspires.ftc.teamcode.subsystems.vision.callReadPipeline;

@TeleOp
public class calibrateVision extends LinearOpMode {

    @Override
    public void runOpMode() {
        callReadPipeline crp = new callReadPipeline(hardwareMap);

        String selectedColor = "Red";
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
            if (gamepad1.a) {selectedColor = "Red";}
            if (gamepad1.b) {selectedColor = "Blue";}
            if (gamepad1.x || gamepad1.y) {selectedColor = "None";}
            if (selectedColor == "Red") {
                OldPipeline.RH_MIN = (int) crp.getH() - 15;
                OldPipeline.RS_MIN = (int) crp.getS() - 30;
                OldPipeline.RV_MIN = (int) crp.getV() - 30;

                OldPipeline.RH_MAX = (int) crp.getH() + 15;
                OldPipeline.RS_MAX = (int) crp.getS() + 30;
                OldPipeline.RV_MAX = (int) crp.getV() + 30;
            } else if (selectedColor == "Blue") {
                OldPipeline.BH_MIN = (int) crp.getH() - 15;
                OldPipeline.BS_MIN = (int) crp.getS() - 30;
                OldPipeline.BV_MIN = (int) crp.getV() - 30;

                OldPipeline.BH_MAX = (int) crp.getH() + 15;
                OldPipeline.BS_MAX = (int) crp.getS() + 30;
                OldPipeline.BV_MAX = (int) crp.getV() + 30;
            }
            if (isStopRequested()) {
                crp.StopPipeline();break;
            }
            telemetry.addData("Selected Color", selectedColor);
            telemetry.addLine("\n   Values:");
            telemetry.addData("H Value", crp.getH());
            telemetry.addData("S Value", crp.getS());
            telemetry.addData("V Value", crp.getV());
            telemetry.update();
        }
    }
}