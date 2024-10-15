package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.Arm1;
import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.Elbow;
import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.ServoDiffyWrist;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.opmode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "1+3")
public class left1plus3 extends LinearOpMode {

    public static int initWrist = -140;

    public static int intakeWrist = -5;
    public static int arm1Intake = -2000;
    public static int arm2Intake = 175;

    public static int arm1Bucket = -1000;
    private static int arm2Bucket = -1800;
    public static int basketWrist = -5;

    public static int intakeRoll = 0;
    public static int lastSampleWristRoll;

    private ServoDiffyWrist diffyWrist;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Arm1 arm1 = (new Arm1(hardwareMap));
        Elbow arm2 = new Elbow(hardwareMap);
        diffyWrist = new ServoDiffyWrist(hardwareMap);
        DcMotor left_lift = hardwareMap.get(DcMotor.class, "left_lift");
        DcMotor right_lift = hardwareMap.get(DcMotor.class, "right_lift");
        DcMotor elbow = hardwareMap.get(DcMotor.class, "elbow");
        DcMotor elbow2 = hardwareMap.get(DcMotor.class, "elbow2");
        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        Servo plane = hardwareMap.get(Servo.class, "plane");

        Pose2d StartPos = new Pose2d(-39.00, -63.00, Math.toRadians(90.00));
        Pose2d BasketPos = new Pose2d(-46.00, -47.00, Math.toRadians(-135.00));
        Pose2d Sample3Pos = new Pose2d(-48.00, -32.00, Math.toRadians(90.00));
        Pose2d Sample2Pos = new Pose2d(-57.00, -32.00, Math.toRadians(90.00));
        Pose2d Sample1Pos = new Pose2d(-56.00, -23.00, Math.toRadians(180.00));
        Pose2d ParkPos = new Pose2d(-24.00, -12.00, Math.toRadians(180.00));

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
                elbow2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elbow2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            left_lift.setPower(-gamepad2.right_stick_y);
            right_lift.setPower(-gamepad2.right_stick_y);
            elbow.setPower(gamepad2.left_stick_y);
            elbow2.setPower(gamepad2.left_stick_y);

            if(gamepad2.dpad_up)
                initWrist -= 0.1;
            if(gamepad2.dpad_down)
                initWrist += 0.1;

            if (gamepad2.right_bumper) {
                intake.setPower(-1);
                plane.setPosition(-1);
            }
            else {
                intake.setPower(0);
                plane.setPosition(0);
            }
            if (gamepad2.left_bumper)
                intake.setPower(1);
            else
                intake.setPower(0);

            diffyWrist.runToProfile(initWrist, 0);

            telemetry.addData("Left Lift Encoder", left_lift.getCurrentPosition());
            telemetry.addData("Right Lift Encoder", right_lift.getCurrentPosition());
            telemetry.addData("Elbow Encoder", elbow.getCurrentPosition());
            telemetry.addData("Elbow2 Encoder", elbow2.getCurrentPosition());
            telemetry.addData("wrist Pos: ", initWrist);
            telemetry.update();
        }

        TrajectorySequence toPreload = drive.trajectorySequenceBuilder(StartPos)
                .lineToLinearHeading(BasketPos)
                .build();
        TrajectorySequence intakeSample3 = drive.trajectorySequenceBuilder(BasketPos)
                .lineToLinearHeading(Sample3Pos)
                .build();
        TrajectorySequence dropSample3 = drive.trajectorySequenceBuilder(Sample3Pos)
                .lineToLinearHeading(BasketPos)
                .build();
        TrajectorySequence intakeSample2 = drive.trajectorySequenceBuilder(BasketPos)
                .lineToLinearHeading(Sample2Pos)
                .build();
        TrajectorySequence dropSample2 = drive.trajectorySequenceBuilder(Sample2Pos)
                .lineToLinearHeading(BasketPos)
                .build();
        TrajectorySequence intakeSample1 = drive.trajectorySequenceBuilder(BasketPos)
                .lineToLinearHeading(Sample1Pos)
                .build();
        TrajectorySequence dropSample1 = drive.trajectorySequenceBuilder(Sample1Pos)
                .lineToLinearHeading(BasketPos)
                .build();
        TrajectorySequence toPark = drive.trajectorySequenceBuilder(BasketPos)
                .lineToLinearHeading(ParkPos)
                .build();


        waitForStart();

        if (!isStopRequested()) {

            drive.setPoseEstimate(StartPos);

            arm1.ArmToPos(arm1Bucket, 0.5);
            arm2.ArmToPos(arm2Bucket, 1);
            diffyWrist.runToProfile(basketWrist, 0);
            sleep(1000);
            drive.followTrajectorySequence(toPreload);

            intake.setPower(-.5);
            plane.setPosition(-.5);
            sleep(500);
            intake.setPower(0);
            plane.setPosition(0);

            arm1.ArmToPos(arm1Intake, 1);
            arm2.ArmToPos(arm2Intake, 0.5);
            diffyWrist.runToProfile(intakeWrist,0);
            drive.followTrajectorySequence(intakeSample3);

            intake.setPower(.5);
            plane.setPosition(.5);
            sleep(500);
            intake.setPower(0);
            plane.setPosition(0);

            arm1.ArmToPos(arm1Bucket, 0.5);
            arm2.ArmToPos(arm2Bucket, 1);
            diffyWrist.runToProfile(basketWrist, 0);
            drive.followTrajectorySequence(dropSample3);

            intake.setPower(-.5);
            plane.setPosition(-.5);
            sleep(500);
            intake.setPower(0);
            plane.setPosition(0);


            arm1.ArmToPos(arm1Intake, 1);
            arm2.ArmToPos(arm2Intake, 0.5);
            diffyWrist.runToProfile(intakeWrist,0);
            drive.followTrajectorySequence(intakeSample2);

            intake.setPower(.5);
            plane.setPosition(.5);
            sleep(500);
            intake.setPower(0);
            plane.setPosition(0);

            arm1.ArmToPos(arm1Bucket, 0.5);
            arm2.ArmToPos(arm2Bucket, 1);
            diffyWrist.runToProfile(basketWrist, 0);
            drive.followTrajectorySequence(dropSample2);

            intake.setPower(-.5);
            plane.setPosition(-.5);
            sleep(500);
            intake.setPower(0);
            plane.setPosition(0);


            arm1.ArmToPos(arm1Intake, 1);
            arm2.ArmToPos(arm2Intake, 0.5);
            diffyWrist.runToProfile(intakeWrist,0);
            drive.followTrajectorySequence(intakeSample1);

            intake.setPower(.5);
            plane.setPosition(.5);
            sleep(500);
            intake.setPower(0);
            plane.setPosition(0);

            arm1.ArmToPos(arm1Bucket, 0.5);
            arm2.ArmToPos(arm2Bucket, 1);
            diffyWrist.runToProfile(basketWrist, 0);
            drive.followTrajectorySequence(dropSample1);

            intake.setPower(-.5);
            plane.setPosition(-.5);
            sleep(500);
            intake.setPower(0);
            plane.setPosition(0);


            arm1.ArmToPos(arm1Intake, 0.45);
            arm2.ArmToPos(arm2Intake, 0.35);
            diffyWrist.runToProfile(intakeWrist,0);
            drive.followTrajectorySequence(toPark);

            sleep(30000);
        }
    }
}