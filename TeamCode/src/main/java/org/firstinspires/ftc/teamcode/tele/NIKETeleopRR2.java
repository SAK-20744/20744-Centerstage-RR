package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm1;
import org.firstinspires.ftc.teamcode.subsystems.ServoArm;

@Config
@TeleOp(name= "NIKE Teleop2 - (Intake initializes near you)", group = "advanced")
public class NIKETeleopRR2 extends LinearOpMode {

    private DcMotor left_lift;
    private DcMotor right_lift;

    private CRServo leftArm;
    private CRServo rightArm;
    private CRServo intake;
    private Servo door;
    private Servo wrist;
    private Servo plane;

    private AnalogInput leftAnalogInput;
    private AnalogInput rightAnalogInput;
    private double leftPos;
    private double rightPos;

    private double power = 1;
    private int targetPos;

    private Arm1 arm1;
    private ServoArm arm2;

    private double currentArmPos = 0;
    private double lastArmPos = 0;
    private double deltaArmPos;

    private double universalArmPos = 0;
    private double uncorrectedArmPos = 0;
    private double correctedArmPos = 0;

    public static double kP;
    public static double kI;
    public static double kD;
    public static double kF;

    private double wristservoposition;

    private boolean ButtonXBlock;
    private boolean ButtonOBlock;

    @Override
    public void runOpMode() throws InterruptedException {

        ButtonXBlock = false;
        ButtonOBlock = false;

        wristservoposition = 0;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(new Pose2d(0,0,0));

        left_lift = hardwareMap.get(DcMotor.class, "left_lift");
        right_lift = hardwareMap.get(DcMotor.class, "right_lift");
        leftAnalogInput = hardwareMap.get(AnalogInput.class, "left");
        rightAnalogInput = hardwareMap.get(AnalogInput.class, "right");
        leftArm = hardwareMap.get(CRServo.class, "leftArm");
        rightArm = hardwareMap.get(CRServo.class, "rightArm");
        intake = hardwareMap.get(CRServo.class, "intake");
        door = hardwareMap.get(Servo.class, "door");
        wrist = hardwareMap.get(Servo.class, "wrist");
        plane = hardwareMap.get(Servo.class, "plane");
        arm1 = (new Arm1(hardwareMap));
        arm2 = (new ServoArm(hardwareMap));

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
//
//            lastArmPos = currentArmPos;
            Pose2d poseEstimate = drive.getPoseEstimate();
//            leftPos = leftAnalogInput.getVoltage() / leftAnalogInput.getMaxVoltage() * 360;
//            rightPos = rightAnalogInput.getVoltage() / rightAnalogInput.getMaxVoltage() * 360;

            Vector2d input = new Vector2d(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());
            
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            if (gamepad2.dpad_left) {
                left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                left_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            left_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            } else if (!gamepad2.circle) {
                ButtonOBlock = false;
            }

            if(gamepad2.right_bumper)
                intake.setPower(1);
            else if(gamepad2.y)
                intake.setPower(-1);
            else
                intake.setPower(0);

            if (gamepad2.left_bumper)
                door.setPosition(0);
            else
                door.setPosition(0.9);

            if (gamepad2.dpad_right) {
                plane.setPosition(0.6);
            } else {
                plane.setPosition(0.2);
            }

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Left Lift Encoder", left_lift.getCurrentPosition());
            telemetry.addData("Right Lift Encoder", left_lift.getCurrentPosition());
            telemetry.addData("Corrected: ", arm2.getLocation());

            arm2.updateServoArm();
            telemetry.update();
        }
    }
}
