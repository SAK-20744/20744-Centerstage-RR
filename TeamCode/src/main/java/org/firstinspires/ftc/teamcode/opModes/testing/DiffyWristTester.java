package org.firstinspires.ftc.teamcode.opModes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.DiffyWrist;

@Config
@TeleOp
public class DiffyWristTester extends LinearOpMode {

    private DiffyWrist diffyWrist;
    public static double desiredPitch = 0;  // Example: 30 degrees
    public static double desiredRoll = 180;

    @Override
    public void runOpMode() {

        // Initialize the DiffyWrist subsystem
        diffyWrist = new DiffyWrist(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            diffyWrist.runToProfile(desiredRoll, desiredPitch);

            while(diffyWrist.isBusy() && !isStopRequested() ) {
                diffyWrist.updateServoArm();
                telemetry.addData("leftPos: ", diffyWrist.getLeftPosition());
                telemetry.addData("rightPos: ", diffyWrist.getRightPosition());
                telemetry.addData("leftTarget: ", diffyWrist.getLeftTarget());
                telemetry.addData("rightTarget: ", diffyWrist.getRightTarget());
                telemetry.addData("Moving?: ", diffyWrist.isBusy());
                telemetry.update();
            }

            // Display telemetry (optional)
            telemetry.addData("Left Position", diffyWrist.getLeftPosition());
            telemetry.addData("Right Position", diffyWrist.getRightPosition());
            telemetry.addData("leftTarget: ", diffyWrist.getLeftTarget());
            telemetry.addData("rightTarget: ", diffyWrist.getRightTarget());
            telemetry.update();

        }
    }
}