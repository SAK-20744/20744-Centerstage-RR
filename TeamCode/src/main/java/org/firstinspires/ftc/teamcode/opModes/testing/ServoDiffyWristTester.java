package org.firstinspires.ftc.teamcode.opModes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.DiffyWrist;
import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.ServoDiffyWrist;

@Config
@Disabled
@TeleOp
public class ServoDiffyWristTester extends LinearOpMode {

    private ServoDiffyWrist diffyWrist;
    public static double desiredPitch = 0;  // Example: 30 degrees
    public static double desiredRoll = 0;

//    public static MultipleTelemetry telemetry;


    @Override
    public void runOpMode() {

        // Initialize the DiffyWrist subsystem
        diffyWrist = new ServoDiffyWrist(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            diffyWrist.runToProfile(desiredPitch, desiredRoll);
            telemetry.addData("Left Pos: ", diffyWrist.getLeftPos());
            telemetry.addData("Right Pos: ", diffyWrist.getRightPos());
            telemetry.addData("Left Servo Pos: ", diffyWrist.getLeftServoPos());
            telemetry.addData("Right Servo Pos: ", diffyWrist.getRightServoPos());
            telemetry.update();
        }
    }
}