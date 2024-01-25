package org.firstinspires.ftc.teamcode.opModes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.DiffyWrist;

@Config
@TeleOp
public class DIffy2Tester extends LinearOpMode {

    private DiffyWrist diffyWrist;
    public static double desiredPitch = 0;  // Example: 30 degrees
    public static double desiredRoll = 180;

    @Override
    public void runOpMode() {

        // Initialize the DiffyWrist subsystem
        diffyWrist = new DiffyWrist(hardwareMap);

        waitForStart();

        diffyWrist.movePitch(desiredPitch);
        diffyWrist.moveRoll(desiredRoll);

        while (opModeIsActive()) {

//            diffyWrist.updateServoArm();

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

            // Display telemetry (optional)
            telemetry.addData("Left Position", diffyWrist.getLeftPosition());
            telemetry.addData("Right Position", diffyWrist.getRightPosition());
            telemetry.addData("leftCorrectedPos: ", diffyWrist.getCorrectedLeftPos());
            telemetry.addData("rightCorrectedPos: ", diffyWrist.getCorrectedRightPos());
            telemetry.addData("leftTarget: ", diffyWrist.getLeftTarget());
            telemetry.addData("rightTarget: ", diffyWrist.getRightTarget());
            telemetry.addData("Moving?: ", diffyWrist.isBusy());
            telemetry.update();

        }
    }
}