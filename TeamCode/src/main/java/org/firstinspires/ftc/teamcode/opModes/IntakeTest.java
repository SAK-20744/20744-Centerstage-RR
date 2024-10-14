package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class IntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        CRServo intake;
//        Servo plane;
        intake = hardwareMap.get(CRServo.class, "intake");
//        plane = hardwareMap.get(Servo.class, "plane");

        waitForStart();
        if (!isStopRequested()) {
            intake.setPower(1);
            sleep(5000);
            intake.setPower(0);
        }
    }
}
