package org.firstinspires.ftc.teamcode.opModes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.DiffyWrist;

@Config
@TeleOp
public class DiffyWristTester extends LinearOpMode {

    private DiffyWrist diffyWrist;
    public static double desiredPitch = 30;  // Example: 30 degrees
    public static double desiredRoll = 90;

//    public static MultipleTelemetry telemetry;


    @Override
    public void runOpMode() {

        // Initialize the DiffyWrist subsystem
        diffyWrist = new DiffyWrist(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {

            diffyWrist.runToProfile(desiredPitch, desiredRoll);

            while(diffyWrist.isBusy() && !isStopRequested() ) {
                diffyWrist.updateServoArm();
                telemetry.addData("leftPos: ", diffyWrist.getLeftPosition());
                telemetry.addData("rightPos: ", diffyWrist.getRightPosition());
                telemetry.addData("leftCorrectedPos: ", diffyWrist.getCorrectedLeftPos());
                telemetry.addData("rightCorrectedPos: ", diffyWrist.getCorrectedRightPos());
                telemetry.addData("leftTarget: ", diffyWrist.getLeftTarget());
                telemetry.addData("rightTarget: ", diffyWrist.getRightTarget());
                telemetry.addData("Moving?: ", diffyWrist.isBusy());
                telemetry.update();
            }

        }
    }
}