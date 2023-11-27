package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.ServoArm;

@Config
@TeleOp
public class TestPID extends OpMode {

    private CRServo leftArm;
    private CRServo rightArm;
    private AnalogInput leftAnalogInput;
    private AnalogInput rightAnalogInput;
    private Servo wrist;

    private PIDFController controller;
    private ServoArm arm2;

    public static double p = 0.0167;
    public static double i = 0.008;
    public static double d = 0.00014;
    public static double f = 0.00065;

    public static int target = 60;

    @Override
    public void init() {

        controller = new PIDFController(p, i, d, f);
        leftArm = hardwareMap.get(CRServo.class, "leftArm");
        rightArm = hardwareMap.get(CRServo.class, "rightArm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm2 = new ServoArm(hardwareMap);
    }

    @Override
    public void loop() {
//        controller.setPIDF(p,i,d,f);
//
//        arm2.updateServoArm();
//        wrist.setPosition(0.63);

        arm2.runToProfile(target);
//
//        double armPower = controller.calculate(arm2.getLocation(), target);

//        telemetry.addData("armPower: ", armPower);
        telemetry.addData("CorrectedArmPos: ", arm2.getLocation());
        telemetry.addData("target", target);
//
//        rightArm.setPower(-armPower);
//        leftArm.setPower(armPower);

    }
}
