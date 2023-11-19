package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    public static double power = 0.5;
    public static int targetPos = 500;

    private Arm1 arm1;

    private double target;
    double error;

    double botHeading;
    BNO055IMU imu;


//    private boolean isClosed = true;
//    private boolean buttonblock = false;

    @Override
    public void init()
    {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        front_left   = hardwareMap.get(DcMotor.class, "fl");
        front_right  = hardwareMap.get(DcMotor.class, "fr");
        back_left    = hardwareMap.get(DcMotor.class, "bl");
        back_right   = hardwareMap.get(DcMotor.class, "br");
        left_lift    = hardwareMap.get(DcMotor.class, "left_lift");
        right_lift   = hardwareMap.get(DcMotor.class, "right_lift");

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

//        telemetry.addData("target angle", target);
//        telemetry.addData("angle error", error);
//        telemetry.addData("IMU", imu.getAngularOrientation().firstAngle);
//        telemetry.addData("botheading", Math.toDegrees(botHeading));
        telemetry.update();
    }

    @Override
    public void loop()
    {
        botHeading = imu.getAngularOrientation().firstAngle;

//        telemetry.addData("target angle", target);
//        telemetry.addData("angle error", error);
//        telemetry.addData("IMU", imu.getAngularOrientation().firstAngle);
//        telemetry.addData("botheading", Math.toDegrees(botHeading));
        telemetry.update();

        double x = -gamepad1.left_stick_y * 1.17; // Correct for imperfect Strafing
        double y = -gamepad1.left_stick_x;
        double rot = gamepad1.right_stick_x;

////        Controls
//        if(gamepad1.a)
//        {
//            target = Math.toDegrees(imu.getAngularOrientation().firstAngle);
//        }

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rot), 1);
        double frontLeftPower = (rotY + rotX + rot) / denominator;
        double backLeftPower = (rotY - rotX + rot) / denominator;
        double frontRightPower = (rotY - rotX - rot) / denominator;
        double backRightPower = (rotY + rotX - rot) / denominator;

        front_left.setPower(frontLeftPower);
        back_left.setPower(backLeftPower);
        front_right.setPower(frontRightPower);
        back_right.setPower(backRightPower);

        arm1.ArmToPos(targetPos, power);

//        if(gamepad2.right_bumper && !buttonblock)
//        {
//            buttonblock = true;
//            if (!isClosed) {
//                claw.setPosition(open);
//                isClosed = true;
//            }
//            else {
//                claw.setPosition(closed);
//                isClosed = false;
//            }
//
//        }
//        else if(!gamepad2.right_bumper)
//        {
//            buttonblock = false;
//        }

    }
}