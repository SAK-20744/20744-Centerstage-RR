package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Arm1;

@Config

@TeleOp(name="NIKE Teleop", group="Iterative Opmode")

public class NikeTeleop extends OpMode
{

    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor left_lift;
    private DcMotor right_lift;

    private CRServo leftArm;
    private CRServo rightArm;

    private CRServo intake;
    private Servo door;
    private Servo wrist;

    private AnalogInput leftAnalogInput;
    private AnalogInput rightAnalogInput;

    private double leftPos;
    private double rightPos;
    private double universalArmPos = 0;
    private double uncorrectedArmPos = 0;
    private double correctedArmPos = 0;

    private int counter;
    private boolean firstTime;
    private boolean secondTime;
    private boolean armButtonBlockPositiveDirection;
    private boolean armButtonBlockNegativeDirection;

    public static double power = 1;
    public static int targetPos;

    private Arm1 arm1;

    private double target;
    double error;

    private double currentArmPos = 0;
    private double lastArmPos = 0;
    private double deltaArmPos;

    double botHeading;
    IMU imu;


//    private boolean isClosed = true;
//    private boolean buttonblock = false;

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
//                        new RevHubOrientationOnRobot(RevHubOrientationOnRobot.xyzOrientation(Math.toDegrees(0),Math.toDegrees(0),Math.toDegrees(-90)))
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                        )
                )
        );

        front_left = hardwareMap.get(DcMotor.class, "fl");
        front_right = hardwareMap.get(DcMotor.class, "fr");
        back_left = hardwareMap.get(DcMotor.class, "bl");
        back_right = hardwareMap.get(DcMotor.class, "br");
        left_lift = hardwareMap.get(DcMotor.class, "left_lift");
        right_lift = hardwareMap.get(DcMotor.class, "right_lift");
        leftAnalogInput = hardwareMap.get(AnalogInput.class, "left");
        rightAnalogInput = hardwareMap.get(AnalogInput.class, "right");
        leftArm = hardwareMap.get(CRServo.class, "leftArm");
        rightArm = hardwareMap.get(CRServo.class, "rightArm");
        intake = hardwareMap.get(CRServo.class, "intake");
        door = hardwareMap.get(Servo.class, "door");
        wrist = hardwareMap.get(Servo.class, "wrist");

        arm1 = (new Arm1(hardwareMap));

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);

        arm1.resetArm1();

        telemetry.update();

    }

    @Override
    public void loop()
    {
        lastArmPos = currentArmPos;

        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        leftPos = leftAnalogInput.getVoltage() / leftAnalogInput.getMaxVoltage() * 360;
        rightPos = rightAnalogInput.getVoltage() / rightAnalogInput.getMaxVoltage() * 360;

        double x = -gamepad1.left_stick_y * 1.17; // Correct for imperfect Strafing
        double y = -gamepad1.left_stick_x;
        double rot = -gamepad1.right_stick_x;

//        Controls
        if(gamepad1.a)
        {
            target = Math.toDegrees(botHeading);
        }

        if (gamepad2.dpad_left) {
            left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        leftArm.setPower(-gamepad2.left_stick_y);
        rightArm.setPower(gamepad2.left_stick_y);

//         Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rot), 1);
        double backRightPower = ((rotY - rotX) + rot) / denominator;
        double backLeftPower = ((rotY + rotX) - rot) / denominator;
        double frontRightPower = ((rotY + rotX) + rot) / denominator;
        double frontLeftPower = ((rotY - rotX) - rot) / denominator;

        front_left.setPower(frontLeftPower);
        back_left.setPower(backLeftPower);
        front_right.setPower(frontRightPower);
        back_right.setPower(backRightPower);

        left_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left_lift.setPower(-gamepad2.right_stick_y);
        right_lift.setPower(-gamepad2.right_stick_y);

        if(gamepad2.left_bumper)
            wrist.setPosition(0);
        else if(gamepad2.right_bumper)
            wrist.setPosition(0.63);
        else
            wrist.setPosition(0.24);

        currentArmPos = rightPos;
        deltaArmPos = lastArmPos - currentArmPos;

        if (Math.abs(deltaArmPos)>90)
        {
            if(rightArm.getPower()<0)
            {
                universalArmPos-=360;
            }
            else
            {
                universalArmPos+=360;
            }
        }
        uncorrectedArmPos = universalArmPos + rightPos;
        correctedArmPos = -1*uncorrectedArmPos/3;

//        if(gamepad2.right_stick_y>0)
//            targetPos+=20;
//        else if(gamepad2.right_stick_y<0)
//            targetPos-=20;
//
//        arm1.ArmToPos(targetPos, power);

        telemetry.addData("target angle", target);
        telemetry.addData("angle error", error);
        telemetry.addData("IMU", botHeading);
        telemetry.addData("botheading", Math.toDegrees(botHeading));
        telemetry.addData("Left Lift Encoder", left_lift.getCurrentPosition());
        telemetry.addData("Right Lift Encoder", left_lift.getCurrentPosition());
        telemetry.addData("Corrected: ", correctedArmPos);
        telemetry.addData("leftServoArm Position: ", leftPos);
        telemetry.addData("rightServoArm Position: ", rightPos);
        telemetry.addData("UniversalArmPos: ", universalArmPos);
        telemetry.addData("lastArmPos: ", lastArmPos);
        telemetry.addData("deltaArmPos: ", deltaArmPos);
        telemetry.addData("rightArmPower: ", rightArm.getPower());
        telemetry.addData("Uncorrected: ", uncorrectedArmPos);

        telemetry.update();

    }
}