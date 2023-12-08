package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp (name = "read calibrated values")
public class readCalibrated extends LinearOpMode {
    static String read;
    @Override
    public void runOpMode() throws InterruptedException {
        new readCalibFile();

        telemetry.addData("Values: ", read);
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            /*if (isStopRequested())
                break;*/
            telemetry.addData("Values: ", read);
            telemetry.update();
        }
    }
}