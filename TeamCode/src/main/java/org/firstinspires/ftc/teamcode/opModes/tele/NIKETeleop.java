package org.firstinspires.ftc.teamcode.opModes.tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.Arm1;
import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.Elbow;
import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.Outake;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.SampleMecanumDrive;

@Disabled
@Config
@TeleOp(name= "NIKE Teleop Old" , group = "advanced")
public class NIKETeleop extends LinearOpMode {

    public static double SPEED = 0.8;
    public static double ELBOWSPEED = 1;

    private double height = 0;
    
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor left_lift;
        DcMotor right_lift;
        DcMotor elbow;

        CRServo intake;
        Servo door;
        Servo wrist;
        Servo plane;

        Arm1 arm1;
        Elbow arm2;

        double looptime = 0;
        boolean ButtonXBlock = false;
        double wristservoposition = 0;
        boolean ButtonOBlock = false;

        boolean intaking = false;
        boolean hanging = false;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(new Pose2d(0,0,0));

        left_lift = hardwareMap.get(DcMotor.class, "left_lift");
        right_lift = hardwareMap.get(DcMotor.class, "right_lift");
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        intake = hardwareMap.get(CRServo.class, "intake");
        door = hardwareMap.get(Servo.class, "door");
        wrist = hardwareMap.get(Servo.class, "wrist");
        plane = hardwareMap.get(Servo.class, "plane");

        arm1 = (new Arm1(hardwareMap));
        arm2 = new Elbow(hardwareMap);

        Outake outake = new Outake(hardwareMap);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
//                        new RevHubOrientationOnRobot(RevHubOrientationOnRobot.xyzOrientation(Math.toDegrees(0),Math.toDegrees(0),Math.toDegrees(-90)))
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                        )
                )
        );

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -(gamepad1.left_stick_y),
                            -(gamepad1.left_stick_x),
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

//            left_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            left_lift.setPower(-gamepad2.right_stick_y * SPEED);
//            right_lift.setPower(-gamepad2.right_stick_y * SPEED);
//            elbow.setPower(gamepad2.left_stick_y * ELBOWSPEED);

//            if (gamepad2.dpad_up)
//                wristservoposition = wristservoposition + 0.01;
//            if (gamepad2.dpad_down)
//                wristservoposition = wristservoposition - 0.01;
//            if (gamepad2.left_trigger > 0)
//                wristservoposition = wristservoposition + 0.01;
//            if (gamepad2.right_trigger > 0)
//                wristservoposition = wristservoposition - 0.01;
//
//            wristservoposition = Math.min(Math.max(wristservoposition, 0), 1);
//            telemetry.addData("servoPosition", wristservoposition);
//            wrist.setPosition(wristservoposition);

//            if (gamepad2.x && !ButtonXBlock) {
//                ButtonXBlock = true;
//                if (wrist.getPosition() == 0.63) {
//                    wristservoposition = 0.24;
//                } else if (wrist.getPosition() == 0.24) {
//                    wristservoposition = 0;
//                } else {
//                    wristservoposition = 0.63;
//                }
//            } else if (!gamepad2.x) {
//                ButtonXBlock = false;
//            }
//            if (gamepad2.circle && !ButtonOBlock) {
//                ButtonOBlock = true;
//            } else if (!gamepad2.circle) {
//                ButtonOBlock = false;
//            }

            if(gamepad2.right_bumper || gamepad1.right_bumper) {
                intake.setPower(-1);
                intaking = true;
            }
            else if(gamepad1.y) {
                intake.setPower(1);
                intaking = true;
            }
            else if(gamepad2.y) {
                intake.setPower(1);
            }
            else {
                intake.setPower(0);
                intaking = false;
            }

            if (gamepad2.left_bumper)
                door.setPosition(0);
            else
                door.setPosition(0.95);

            if (gamepad2.dpad_right) {
                plane.setPosition(0.6);
            } else {
                plane.setPosition(0.2);
            }

            height -= gamepad2.left_stick_y*0.6;

//            if(gamepad2.a)
//                height = 0;
//
//            if(gamepad2.b){
//                hanging = true;
//            }
//            else{
//                hanging = false;
//            }

//            if(gamepad2.x && intaking == false) {
//                intaking = true;
//                sleep(50);
//            }
//            else if(intaking == true && gamepad2.x) {
//                intaking = false;
//                sleep(50);
//            }

            outake.BackdropHeightOffset10(height, intaking, hanging);

            telemetry.addData("Height:", height);
            telemetry.addData("Arm1 Degrees:" , outake.getMotorArmDeg());
            telemetry.addData("Arm1 Pos:" , outake.getMotorArmPos());
            telemetry.addData("Arm2 Degrees:" , outake.getServoArmDeg());
            telemetry.addData("Arm2 Pos:" , outake.getServoArmPos());
            telemetry.addData("Wrist Degrees:" , outake.getWristDeg());
            telemetry.addData("Wrist Pos:" , outake.getWristPos());
//            telemetry.addData("IMU", imu.getRobotYawPitchRollAngles());
//            telemetry.addData("Left Lift Encoder", left_lift.getCurrentPosition());
//            telemetry.addData("Right Lift Encoder", left_lift.getCurrentPosition());
//            telemetry.addData("Elbow Encoder", elbow.getCurrentPosition());

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - looptime));
            looptime = loop;


            telemetry.update();
        }
    }
}
