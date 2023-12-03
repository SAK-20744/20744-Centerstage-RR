package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Arm1;
import org.firstinspires.ftc.teamcode.subsystems.IVKMath;
import org.firstinspires.ftc.teamcode.subsystems.Outake;
import org.firstinspires.ftc.teamcode.subsystems.ServoArm;

@Config
//@Disabled
@TeleOp
public class TestPID extends OpMode {

    private CRServo leftArm;
    private CRServo rightArm;
    private AnalogInput leftAnalogInput;
    private AnalogInput rightAnalogInput;
    private Servo wrist;

    private PIDFController controller;
    private ServoArm arm2;
    private Arm1 arm1;

    private IVKMath ivk;
    private Outake deposit;

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
        arm1 = new Arm1(hardwareMap);
        deposit = new Outake(hardwareMap);

    }

    @Override
    public void loop() {
        deposit.BackdropHeight(target);
    }

}
