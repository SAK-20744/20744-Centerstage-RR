package org.firstinspires.ftc.teamcode.tele;

        import com.qualcomm.hardware.bosch.BHI260IMU;
//        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.AnalogInput;
        import com.qualcomm.robotcore.hardware.CRServo;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.IMU;
        import com.qualcomm.robotcore.hardware.Servo;
        import org.firstinspires.ftc.robotcore.external.JavaUtil;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "NIKETeleop2 (Blocks to Java)")
public class NIKETeleop2 extends LinearOpMode {

    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private Servo wrist;
    private DcMotor left_lift;
    private DcMotor right_lift;
    private CRServo leftArm;
    private CRServo rightArm;
    private CRServo intake;
    private Servo door;
    private AnalogInput left;
    private AnalogInput right;

    double botHeading;
    IMU imu;

    int inState;

    /**
     * Describe this function...
     */

    private void drive() {

        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        botHeading = robotOrientation.getYaw(AngleUnit.DEGREES);

        double x = -gamepad1.left_stick_y * 1.17; // Correct for imperfect Strafing
        double y = gamepad1.left_stick_x;
        double rot = gamepad1.right_stick_x;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

//        double denominator = Math.max((Math.abs(rotY) + Math.abs(rotX) + Math.abs(rot)), 1);
        double denominator = 1;
        double frontLeftPower = (rotY + rotX + rot) / denominator;
        double backLeftPower = (rotY - rotX + rot) / denominator;
        double frontRightPower = (rotY - rotX - rot) / denominator;
        double backRightPower = (rotY + rotX - rot) / denominator;

        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        fr.setPower(frontRightPower);
        br.setPower(backRightPower);

        telemetry.addData("fl", frontLeftPower);
        telemetry.addData("bl", backLeftPower);
        telemetry.addData("fr", frontRightPower);
        telemetry.addData("br", backRightPower);

        telemetry.addData("angle", botHeading);

        telemetry.update();

    }

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        boolean ButtonXBlock = false;
        double wristservoposition;
        boolean ButtonOBlock = false;

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

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        wrist = hardwareMap.get(Servo.class, "wrist");
        left_lift = hardwareMap.get(DcMotor.class, "left_lift");
        right_lift = hardwareMap.get(DcMotor.class, "right_lift");
        leftArm = hardwareMap.get(CRServo.class, "leftArm");
        rightArm = hardwareMap.get(CRServo.class, "rightArm");
        intake = hardwareMap.get(CRServo.class, "intake");
        door = hardwareMap.get(Servo.class, "door");
        wrist = hardwareMap.get(Servo.class, "wrist");
        left = hardwareMap.get(AnalogInput.class, "left");
        right = hardwareMap.get(AnalogInput.class, "right");

        wristservoposition = 0.3;
        wristservoposition = 0.63;
        // move motors to -660 for init
        wrist.setPosition(wristservoposition);
        // Put initialization blocks here.
        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);

        while (!(opModeIsActive() || isStopRequested())) {
            left_lift.setPower(-gamepad2.right_stick_y);
            right_lift.setPower(-gamepad2.right_stick_y);
            leftArm.setPower(-gamepad2.left_stick_y);
            rightArm.setPower(gamepad2.left_stick_y);
            telemetry.addData("Left Lift Encoder", left_lift.getCurrentPosition());
            telemetry.addData("Right Lift Encoder", left_lift.getCurrentPosition());
            telemetry.update();
            if (gamepad2.a) {
                left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                left_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                drive();
                if (gamepad2.right_bumper) {
                    intake.setPower(1);
                } else {
                    intake.setPower(0);
                }
                left_lift.setPower(-gamepad2.right_stick_y);
                right_lift.setPower(-gamepad2.right_stick_y);
                leftArm.setPower(-gamepad2.left_stick_y);
                rightArm.setPower(gamepad2.left_stick_y);
                if (gamepad2.dpad_up) {
                    wristservoposition = wristservoposition + 0.01;
                }
                if (gamepad2.dpad_down) {
                    wristservoposition = wristservoposition - 0.01;
                }
                wristservoposition = Math.min(Math.max(wristservoposition, 0), 1);
                telemetry.addData("servoPosition", wristservoposition);
                wrist.setPosition(wristservoposition);
                if (gamepad2.left_bumper) {
                    door.setPosition(0);
                } else {
                    door.setPosition(0.65);
                }
                telemetry.addData("LeftLift", left_lift.getCurrentPosition());
                telemetry.addData("RightLift", right_lift.getCurrentPosition());
                telemetry.addData("leftPos", (left.getVoltage() / 3.3) * 360);
                telemetry.addData("rightPos", (right.getVoltage() / 3.3) * 360);
                if (gamepad2.x && !ButtonXBlock) {
                    ButtonXBlock = true;
                    if (wrist.getPosition() == 0.63) {
                        wristservoposition = 0.24;
                    } else if (wrist.getPosition() == 0.24) {
                        wristservoposition = 0;
                    } else {
                        wristservoposition = 0.63;
                    }
                } else if (!gamepad2.x) {
                    ButtonXBlock = false;
                }
                if (gamepad2.circle && !ButtonOBlock) {
                    ButtonOBlock = true;
                    inState = 0;
                } else if (!gamepad2.circle) {
                    ButtonOBlock = false;
                }

                telemetry.addData("angle", botHeading);

                telemetry.update();
                // Put loop blocks here.
            }
        }
    }

    /**
     * Describe this function...
     */
    private void IntakeMachine() {
        if (inState == 0) {
            left_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_lift.setTargetPosition(-2000);
            right_lift.setTargetPosition(-2000);
            left_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            inState = 1;
        }
        if (inState == 1) {
        }
    }
}