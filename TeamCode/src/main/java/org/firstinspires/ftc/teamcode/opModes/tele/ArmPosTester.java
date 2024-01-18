package org.firstinspires.ftc.teamcode.opModes.tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.Arm1;
import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.Elbow;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.SampleMecanumDrive;

@Config
@TeleOp(name= "Arm Pos Tester" , group = "advanced")
public class ArmPosTester extends LinearOpMode {

    public static double SPEED = 0.8;
    public static double ELBOWSPEED = 1;
    
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

        double arm1Deg, arm2Deg;

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

        while (opModeInInit()) {

            if (gamepad2.a) {
                left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                left_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if(gamepad2.b) {
                elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            left_lift.setPower(-gamepad2.right_stick_y);
            right_lift.setPower(-gamepad2.right_stick_y);
            elbow.setPower(gamepad2.left_stick_y);
            wrist.setPosition(0.63);

//            telemetry.addData("Parallel: ", parallelEncoder.getCurrentPosition());
//            telemetry.addData("Perpendicular: ", perpendicularEncoder.getCurrentPosition());
            telemetry.addData("Left Lift Encoder", left_lift.getCurrentPosition());
            telemetry.addData("Right Lift Encoder", right_lift.getCurrentPosition());
            telemetry.addData("Elbow Encoder", elbow.getCurrentPosition());
//            telemetry.addData("imu", imu.getRobotAngularVelocity(AngleUnit.DEGREES));

            telemetry.update();
        }


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            left_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            left_lift.setPower(-gamepad2.right_stick_y * SPEED);
            right_lift.setPower(-gamepad2.right_stick_y * SPEED);
            elbow.setPower(gamepad2.left_stick_y * ELBOWSPEED);

            if (gamepad2.dpad_up)
                wristservoposition = wristservoposition + 0.01;
            if (gamepad2.dpad_down)
                wristservoposition = wristservoposition - 0.01;
            if (gamepad2.left_trigger > 0)
                wristservoposition = wristservoposition + 0.01;
            if (gamepad2.right_trigger > 0)
                wristservoposition = wristservoposition - 0.01;

            wristservoposition = Math.min(Math.max(wristservoposition, 0), 1);
            telemetry.addData("servoPosition", wristservoposition);
            wrist.setPosition(wristservoposition);

            telemetry.addData("Left Lift Encoder", left_lift.getCurrentPosition());
//            telemetry.addData("Right Lift Encoder", left_lift.getCurrentPosition());
            telemetry.addData("Elbow Encoder", elbow.getCurrentPosition());
//            telemetry.addData("Wrist Pos", wristservoposition);
            arm1Deg = (left_lift.getCurrentPosition()/(-1000 / 90));
            arm2Deg = (elbow.getCurrentPosition()/(-1000 / 90));
            telemetry.addData("Arm1 Degrees", arm1Deg);
            telemetry.addData("Arm2 Degrees", arm2Deg);

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - looptime));
            looptime = loop;

            telemetry.update();
        }
    }
}
