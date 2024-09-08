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
@Autonomous(name = "1+2")
public class Bucket1plus2 extends LinearOpMode {

    public static int intakeWrist = -85;
    public static int arm1Intake = -2000;
    public static int arm2Intake = 145;

    public static int arm1Bucket = -1000;
    private static int arm2Bucket = -900;
    public static int basketWrist = -50;

    public static int initWrist = -140;


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

        Pose2d StartPos = new Pose2d(-39.00, -63.00, Math.toRadians(90.00));
        Pose2d BasketPos = new Pose2d(-60.00, -60.00, Math.toRadians(-135.00));
        Pose2d Sample3Pos = new Pose2d(-48.00, -33.00, Math.toRadians(90.00));
        Pose2d Sample2Pos = new Pose2d(-60.00, -33.00, Math.toRadians(90.00));
        Pose2d ParkPos = new Pose2d(60.00, -60.00, Math.toRadians(90));

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
        TrajectorySequence toPark = drive.trajectorySequenceBuilder(BasketPos)
                .lineToLinearHeading(ParkPos)
                .build();


        waitForStart();

        if (!isStopRequested()) {

            drive.setPoseEstimate(StartPos);

            drive.followTrajectorySequence(toPreload);
            arm1.ArmToPos(arm1Bucket, 0.5);
            arm2.ArmToPos(arm2Bucket, 1);
            diffyWrist.runToProfile(basketWrist, 0);

            drive.followTrajectorySequence(intakeSample3);
            arm1.ArmToPos(arm1Intake, 0.45);
            arm2.ArmToPos(arm2Intake, 0.35);
            diffyWrist.runToProfile(intakeWrist,0);

            drive.followTrajectorySequence(dropSample3);
            arm1.ArmToPos(arm1Bucket, 0.5);
            arm2.ArmToPos(arm2Bucket, 1);
            diffyWrist.runToProfile(basketWrist, 0);

            drive.followTrajectorySequence(intakeSample2);
            arm1.ArmToPos(arm1Intake, 0.45);
            arm2.ArmToPos(arm2Intake, 0.35);
            diffyWrist.runToProfile(intakeWrist,0);

            drive.followTrajectorySequence(dropSample2);
            arm1.ArmToPos(arm1Bucket, 0.5);
            arm2.ArmToPos(arm2Bucket, 1);
            diffyWrist.runToProfile(basketWrist, 0);

            drive.followTrajectorySequence(toPark);
            arm1.ArmToPos(arm1Intake, 0.45);
            arm2.ArmToPos(arm2Intake, 0.35);
            diffyWrist.runToProfile(intakeWrist,0);

            sleep(30000);
        }
    }
}