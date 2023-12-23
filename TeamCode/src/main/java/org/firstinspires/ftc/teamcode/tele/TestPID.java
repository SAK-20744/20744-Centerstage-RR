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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Arm1;
import org.firstinspires.ftc.teamcode.subsystems.IVKMath;
import org.firstinspires.ftc.teamcode.subsystems.Outake;
import org.firstinspires.ftc.teamcode.subsystems.ServoArm;

@Config
@Disabled
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

    public static int target = 10;

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

        DcMotor left_lift = hardwareMap.get(DcMotor.class, "left_lift");
        DcMotor right_lift = hardwareMap.get(DcMotor.class, "right_lift");
        CRServo leftArm = hardwareMap.get(CRServo.class, "leftArm");
        CRServo rightArm = hardwareMap.get(CRServo.class, "rightArm");
        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        Servo door = hardwareMap.get(Servo.class, "door");


            left_lift.setPower(-gamepad2.right_stick_y);
            right_lift.setPower(-gamepad2.right_stick_y);
            leftArm.setPower(-gamepad2.left_stick_y);
            rightArm.setPower(gamepad2.left_stick_y);
            telemetry.addData("Left Lift Encoder", left_lift.getCurrentPosition());
            telemetry.addData("Right Lift Encoder", right_lift.getCurrentPosition());
            telemetry.addData("Servo Arm", arm2.getLocation());
            telemetry.update();
            if (gamepad2.a) {
                left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                left_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }
    }

    @Override
    public void loop() {
        deposit.BackdropHeight(target);

        telemetry.addData("ServoArmTargetPos: ", deposit.getServoArmPos());
        telemetry.addData("ServoArmPos: ", arm2.getLocation());
        telemetry.addData("MotorArmTargetPos: ", deposit.getMotorArmPos());
        telemetry.addData("MotorArmPos: ", arm1.getLeftArm1Position());
        telemetry.addData("WristArmTargetPos: ", deposit.getWristPos());
        telemetry.addData("WristArmPos: ", wrist.getPosition());

        telemetry.addData("length", target);

        telemetry.update();

    }

}
