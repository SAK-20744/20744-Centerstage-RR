package org.firstinspires.ftc.teamcode.opModes.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TestingPP extends LinearOpMode {

    private DcMotor LeftFront;
    private DcMotor LeftBack;
    private DcMotor RightFront;
    private DcMotor RightBack;

    @Override
    public void runOpMode() throws InterruptedException {
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");

        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;
        double strafe;
        double move;
        double rotate;


        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            strafe = gamepad1.left_stick_x;
            move = gamepad1.left_stick_y;
            rotate = gamepad1.right_stick_x;
        double ly = -gamepad1.left_stick_y;
        double lx = gamepad1.left_stick_x * 1.2 ;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1);
            double frontLeftPower = (ly + lx + rx) / denominator;
            double backLeftPower = (ly - lx + rx) / denominator;
            double frontRightPower = (ly - lx - rx) / denominator;
            double backRightPower = (ly + lx - rx) / denominator;


            rightFrontPower = 0;
            rightBackPower = 0;
            leftFrontPower = 0;
            leftBackPower = 0;

            LeftBack.setPower(leftBackPower);
            LeftFront.setPower(leftFrontPower);
            RightBack.setPower(rightBackPower);
            RightFront.setPower(rightFrontPower);


        }
    }
}
