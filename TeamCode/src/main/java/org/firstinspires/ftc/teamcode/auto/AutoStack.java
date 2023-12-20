package org.firstinspires.ftc.teamcode.auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm1;
import org.firstinspires.ftc.teamcode.subsystems.ServoArm;

@Autonomous
public class AutoStack extends LinearOpMode {

    @Override
    public void runOpMode() {



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Arm1 arm1 = (new Arm1(hardwareMap));
        ServoArm arm2 = new ServoArm(hardwareMap);

        Trajectory tin = drive.trajectoryBuilder(new Pose2d())
                .forward(20)
                .build();

        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        DcMotor left_lift = hardwareMap.get(DcMotor.class, "left_lift");
        DcMotor right_lift = hardwareMap.get(DcMotor.class, "right_lift");
        CRServo leftArm = hardwareMap.get(CRServo.class, "leftArm");
        CRServo rightArm = hardwareMap.get(CRServo.class, "rightArm");
        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        Servo door = hardwareMap.get(Servo.class, "door");

        boolean ButtonXBlock = false;
        double wristservoposition = 0.63;

        // move motors to -660 for init
        wrist.setPosition(wristservoposition);

        while (!(opModeIsActive() || isStopRequested())) {
            left_lift.setPower(-gamepad2.right_stick_y);
            right_lift.setPower(-gamepad2.right_stick_y);
            leftArm.setPower(-gamepad2.left_stick_y);
            rightArm.setPower(gamepad2.left_stick_y);
            telemetry.addData("Left Lift Encoder", left_lift.getCurrentPosition());
            telemetry.addData("Right Lift Encoder", right_lift.getCurrentPosition());
            telemetry.addData("Servo Arm", arm2.getLocation());
            telemetry.update();
            if (gamepad2.a) {
                left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                left_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }


        waitForStart();

        if (!isStopRequested()) {



            wrist.setPosition(0.16);
            sleep(500);
            drive.followTrajectory(tin);
            sleep(500);
            intake.setPower(-0.3);
            sleep(500);
            intake.setPower(0);
            sleep(700);
            wrist.setPosition(0.14);
            sleep(500);
            intake.setPower(-0.3);
            sleep(500);
            intake.setPower(0);

            sleep(1000);
            leftArm.setPower(0.45);
            rightArm.setPower(-0.45);
            sleep(300);
            leftArm.setPower(0);
            rightArm.setPower(0);
            door.setPosition(0);
            sleep(2000);
            door.setPosition(0.95);
            sleep(500);
//start

        }
    }
}