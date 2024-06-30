package org.firstinspires.ftc.teamcode.opModes.noobRobot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name= "Positive-Protons" , group = "advanced")
public class TeleV1 extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.get(DcMotor.class, "front-left");
        frontRight = hardwareMap.get(DcMotor.class, "front-right");
        backLeft = hardwareMap.get(DcMotor.class, "back-left");
        backRight = hardwareMap.get(DcMotor.class, "back-right");
        //Setting Motors

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        //Making them move in the correct direction
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
        //The loop
            double rx = -gamepad1.right_stick_x;
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
        //Inputs
            //x is strafe left-right
            //y is forward-back
            //rx is rotation
            frontLeft.setPower(-rx + x - y);
            backLeft.setPower(-rx - x - y);
            frontRight.setPower(-rx - x + y);
            backRight.setPower(-rx + x + y);

            if (gamepad2.a) backRight.setPower(1);
            if (gamepad2.b) backLeft.setPower(-1);
            if (gamepad2.x) frontRight.setPower(-1);
            if (gamepad2.y) frontLeft.setPower(1);
        }
    }
}
