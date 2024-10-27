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
@Autonomous(name = "park")
public class parkAuto extends LinearOpMode {

    public static int initWrist = 165;

    public static int intakeWrist = -5;
    public static int arm1Intake = -2000;
    public static int arm2Intake = 0;

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
        Pose2d ParkPos = new Pose2d(-39.00, -12.00, Math.toRadians(180.00));
        Pose2d FinalPos = new Pose2d(-20.00, -12.00, Math.toRadians(180.00));

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

        TrajectorySequence toPark = drive.trajectorySequenceBuilder(StartPos)
                .lineToLinearHeading(ParkPos)
                .lineToLinearHeading(FinalPos)
                .build();


        waitForStart();

        if (!isStopRequested()) {

            drive.setPoseEstimate(StartPos);

            arm1.ArmToPos(arm1Intake, 0.45);
            arm2.ArmToPos(arm2Intake, 0.35);
            diffyWrist.runToProfile(intakeWrist,0);
            drive.followTrajectorySequence(toPark);

            sleep(30000);
        }
    }
}