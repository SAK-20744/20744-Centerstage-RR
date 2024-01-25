package org.firstinspires.ftc.teamcode.opModes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.InverseKinematics.DiffyWrist;

//@Disabled
@Autonomous(name = "Diffy Test Auto")
public class DiffyAutoTesting extends LinearOpMode {

    private DiffyWrist diffyWrist;

    @Override
    public void runOpMode() throws InterruptedException {

        diffyWrist = new DiffyWrist(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        while (opModeInInit()) {

//            telemetry.addData("Parallel: ", parallelEncoder.getCurrentPosition());
//            telemetry.addData("Perpendicular: ", perpendicularEncoder.getCurrentPosition());
//            telemetry.addData("imu", imu.getRobotAngularVelocity(AngleUnit.DEGREES));

            telemetry.update();
        }

        waitForStart();

        if (!isStopRequested()) {

            diffyWrist.runToProfile(0, 0);

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

            sleep( 2000);

            diffyWrist.runToProfile(0, 180);

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

            sleep(2000);

            diffyWrist.runToProfile(90, 180);

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

            sleep(2000);

            diffyWrist.runToProfile(-90, 180);

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

            sleep(2000);

            diffyWrist.runToProfile(45, 0);

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

            sleep(2000);

            diffyWrist.runToProfile(75, 45);

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

            sleep(2000);

            diffyWrist.runToProfile(-45, 135);

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

            sleep(30000);
        }
    }
}