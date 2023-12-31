package org.firstinspires.ftc.teamcode.opModes.tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.Elbow;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.Arm1;
import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.Outake;
import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.ServoArm;

@Config
@Disabled
@TeleOp(name= "NIKE TeleopIVK", group = "advanced")
public class NIKETeleopIVK extends LinearOpMode {

    private DcMotor left_lift;
    private DcMotor right_lift;
    private DcMotor elbow;

    private CRServo leftArm;
    private CRServo rightArm;
    private CRServo intake;
    private Servo door;
    private Servo wrist;
    private Servo plane;

    private Arm1 arm1;
    private Elbow arm2;

    public static double kP;
    public static double kI;
    public static double kD;
    public static double kF;

    private double wristservoposition;

    private boolean ButtonXBlock;
    private boolean ButtonOBlock;

    private double height = 0;

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
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        intake = hardwareMap.get(CRServo.class, "intake");
        door = hardwareMap.get(Servo.class, "door");
        wrist = hardwareMap.get(Servo.class, "wrist");
        plane = hardwareMap.get(Servo.class, "plane");

//        Wrist wrist = new Wrist(hardwareMap);
        arm1 = (new Arm1(hardwareMap));
        arm2 = (new Elbow(hardwareMap));
        Outake outake = new Outake(hardwareMap);



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            input.getX(),
//                            input.getY(),
//                            -gamepad1.right_stick_x
//                    )
//            );
//            drive.update();
//
//            if (gamepad2.dpad_up) {
//                wristservoposition = wristservoposition + 0.01;
//            }
//            if (gamepad2.dpad_down) {
//                wristservoposition = wristservoposition - 0.01;
//            }
//            wristservoposition = Math.min(Math.max(wristservoposition, 0), 1);
//            telemetry.addData("servoPosition", wristservoposition);
//            wrist.setPosition(wristservoposition);
//
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
//
//            if(gamepad2.right_bumper)
//                intake.setPower(1);
//            else if(gamepad2.y)
//                intake.setPower(-1);
//            else
//                intake.setPower(0);
//
//            if (gamepad2.left_bumper)
//                door.setPosition(0);
//            else
//                door.setPosition(0.9);
//
//            if (gamepad2.dpad_right) {
//                plane.setPosition(0.6);
//            } else {
//                plane.setPosition(0.2);
//            }
//
            height -= gamepad2.left_stick_y*0.25;
            if(height>33)
                height = 33;
            if(height<0)
                height = 0;
            outake.BackdropHeight(height, true);

            telemetry.addData("Height:", height);
            telemetry.addData("Arm1 Degrees:" , outake.getMotorArmDeg());
            telemetry.addData("Arm1 Pos:" , outake.getMotorArmPos());
            telemetry.addData("Arm2 Degrees:" , outake.getServoArmDeg());
            telemetry.addData("Arm2 Pos:" , outake.getServoArmPos());
            telemetry.addData("Wrist Degrees:" , outake.getWristDeg());
            telemetry.addData("Wrist Pos:" , outake.getWristPos());
//            telemetry.addData("A1", outake.getA1(height));
//            telemetry.addData("Q1", outake.getQ1(height));
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
//            telemetry.addData("Left Lift Encoder", left_lift.getCurrentPosition());
//            telemetry.addData("Right Lift Encoder", left_lift.getCurrentPosition());
//            telemetry.addData("Corrected: ", arm2.getLocation());

            telemetry.update();
        }
    }
}
