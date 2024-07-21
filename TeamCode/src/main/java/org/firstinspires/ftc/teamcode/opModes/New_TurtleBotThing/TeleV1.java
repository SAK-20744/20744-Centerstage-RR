package org.firstinspires.ftc.teamcode.opModes.New_TurtleBotThing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name= "Turtle Bot" , group = "advanced")
public class TeleV1 extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor IntakeMotor;

    public void runOpMode() throws InterruptedException {
// Setup code
        frontLeft = hardwareMap.get(DcMotor.class, "front-left");
        frontRight = hardwareMap.get(DcMotor.class, "front-right");
        backLeft = hardwareMap.get(DcMotor.class, "back-left");
        backRight = hardwareMap.get(DcMotor.class, "back-right");
        IntakeMotor = hardwareMap.get(DcMotor.class, "intake");
        //Setting Motors to names

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        //Making the motors move in the correct direction
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            //The loop
            double rx = gamepad1.right_stick_x;
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            //Inputs
            //x is strafe left-right
            //y is forward-back
            //rx is rotation
            frontLeft.setPower(y + x + rx);
            backLeft.setPower(y - x + rx);
            frontRight.setPower(y - x - rx);
            backRight.setPower(y + x - rx);

            if (gamepad2.a) backRight.setPower(1);
            if (gamepad2.b) backLeft.setPower(-1);
            if (gamepad2.x) frontRight.setPower(-1);
            if (gamepad2.y) frontLeft.setPower(1);
            if (gamepad1.a) {
                IntakeMotor.setPower(1);
            }
            if (!gamepad1.a) {
                IntakeMotor.setPower(0);
            }
            if (gamepad1.b) {

            }
        }
    }
}
