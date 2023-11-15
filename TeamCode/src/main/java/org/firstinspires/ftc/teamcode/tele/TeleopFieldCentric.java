package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.horizontal;
import org.firstinspires.ftc.teamcode.subsystems.lift;
import org.firstinspires.ftc.teamcode.subsystems.turret;

@TeleOp(name="TeleopFieldCentricBATTERYHOLDERSIDESHOULDNBENEARYOU", group="Iterative Opmode")

public class TeleopFieldCentric extends OpMode
{

    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor liftmotor;

    private DigitalChannel liftswitch;

//    public static double target = -11;
    private double target;
    double error;
    private boolean liftLimit;
    private double liftPower;
    public static double Kp = 0.034;
    public static double tolerance = 0.5;

    double closed = 0;
    double open = 0.45;

    boolean liftLimitReset = true;

    double up = 0;
    double down = 0.2;

    double botHeading;
    BNO055IMU imu;
    turret Turret;
    horizontal Horizontal;
    lift Lift;

    Servo claw;
    Servo wrist;

    private boolean isClosed = true;
    private boolean buttonblock = false;

    private boolean isUp = true;
    private boolean buttonblock2 = false;

    @Override
    public void init()
    {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        front_left   = hardwareMap.get(DcMotor.class, "leftFront");
        front_right  = hardwareMap.get(DcMotor.class, "rightFront");
        back_left    = hardwareMap.get(DcMotor.class, "leftRear");
        back_right   = hardwareMap.get(DcMotor.class, "rightRear");
        liftswitch = hardwareMap.get(DigitalChannel.class, "lift switch");
        liftmotor = hardwareMap.get(DcMotor.class, "lift");

        claw = hardwareMap.get(Servo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");

        Turret = new turret(hardwareMap);
        Lift = new lift(hardwareMap);
        Horizontal = new horizontal(hardwareMap);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);

        Turret.updateTurret();
        Lift.updatelift();

        telemetry.addData("target angle", target);
        telemetry.addData("angle error", error);
        telemetry.addData("IMU", imu.getAngularOrientation().firstAngle);
        telemetry.addData("botheading", Math.toDegrees(botHeading));
//        telemetry.addData("Turret Encoder Position" , Turret.whereAmI());
//        telemetry.addData("Lift Encoder Position" , Lift.whereAmI());
        telemetry.update();

    }

    @Override
    public void loop()
    {

        botHeading = imu.getAngularOrientation().firstAngle;

        telemetry.addData("target angle", target);
        telemetry.addData("angle error", error);
        telemetry.addData("IMU", imu.getAngularOrientation().firstAngle);
        telemetry.addData("botheading", Math.toDegrees(botHeading));
        telemetry.addData("Claw Closed?", isClosed);
//        telemetry.addData("Turret Encoder Position" , Turret.whereAmI());
//        telemetry.addData("Lift Encoder Position" , Lift.whereAmI());
        telemetry.addData("Horizontal", Horizontal.getPosition());
        telemetry.update();

        double x = -gamepad1.left_stick_y * 1.17; // Correct for imperfect Strafing
        double y = -gamepad1.left_stick_x;
        double rot = gamepad1.right_stick_x;        

//        Controls
        if(gamepad1.a)
        {
            target = Math.toDegrees(imu.getAngularOrientation().firstAngle);
        }


        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

//        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rot), 1);
        double frontLeftPower = (rotY + rotX + rot) / denominator;
        double backLeftPower = (rotY - rotX + rot) / denominator;
        double frontRightPower = (rotY - rotX - rot) / denominator;
        double backRightPower = (rotY + rotX - rot) / denominator;


        //   Alignment code
//        // Denominator is the largest motor power (1 or -1)
//        double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
//        double frontLeftPower = (y + x) / denominator;
//        double backLeftPower = (y - x) / denominator;
//        double frontRightPower = (y - x) / denominator;
//        double backRightPower = (y + x) / denominator;
//
//        error = Math.toDegrees(botHeading) - (target);
//
//        if(Math.abs(error)<=tolerance) {
//            error = 0;
//        }
//
//        frontLeftPower += error * Kp;
//        backLeftPower += error * Kp;
//        frontRightPower += -error * Kp;
//        backRightPower += -error * Kp;
//
        front_left.setPower(frontLeftPower);
        back_left.setPower(backLeftPower);
        front_right.setPower(frontRightPower);
        back_right.setPower(backRightPower);

        if(gamepad2.right_bumper && !buttonblock)
        {
            buttonblock = true;
            if (!isClosed) {
                claw.setPosition(open);
                isClosed = true;
            }
            else {
                claw.setPosition(closed);
                isClosed = false;
            }

        }
        else if(!gamepad2.right_bumper)
        {
            buttonblock = false;
        }

        if(gamepad2.left_bumper && !buttonblock2)
        {
            buttonblock2 = true;
            if (!isUp) {
                wrist.setPosition(down);
                isUp = true;
            }
            else {
                wrist.setPosition(up);
                isUp = false;
            }

        }
        else if(!gamepad2.right_bumper)
        {
            buttonblock2 = false;
        }
//
//        Lift.move(gamepad2.right_stick_y);
        Turret.move(-gamepad2.left_stick_x);

        liftPower = -gamepad2.right_stick_y;

        if (liftswitch.getState()){
            if(liftPower<0)
                liftPower = 0;
        }

        liftmotor.setPower(liftPower);

        if (gamepad2.left_stick_y>0.9)
        {
            Horizontal.move(-0.0135);
        }
        else if (gamepad2.left_stick_y<-0.9)
        {
            Horizontal.move(0.0135);
        }
        else if (gamepad2.left_stick_y>0.55)
        {
            Horizontal.move(-0.005);
        }
        else if (gamepad2.left_stick_y<-0.55)
        {
            Horizontal.move(0.005);
        }
        else if (gamepad2.left_stick_y>0)
        {
            Horizontal.move(-0.0017);
        }
        else if (gamepad2.left_stick_y<0)
        {
            Horizontal.move(0.0017);
        }
        Turret.updateTurret();

    }
}