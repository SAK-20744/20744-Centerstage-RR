package org.firstinspires.ftc.teamcode.subsystems.InverseKinematics;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDFController;

@Config
public class DiffyWrist {

    // Define your CRServo objects
    private CRServo leftServo;
    private CRServo rightServo;

    // Define analog encoder objects
    private AnalogInput leftEncoder;
    private AnalogInput rightEncoder;

    // Desired pitch and roll angles
    private double desiredPitch = 0.0;  // Set your desired pitch angle
    private double desiredRoll = 180;   // Set your desired roll angle

    private PIDFController controller;

    private double leftPos;
    private double rightPos;

    private double targetLeft;
    private double targetRight;

    private boolean moving, rightMoving, leftMoving;

    public static double p = 0.0077875;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public static double range = 5;

    private double lastLeftPos, lastRightPos, currentLeftPos, currentRightPos, deltaLeftPos, deltaRightPos, universalRightPos, universalLeftPos;

    public DiffyWrist(HardwareMap hardwareMap) {

        leftServo = hardwareMap.crservo.get("leftServo");
        rightServo = hardwareMap.crservo.get("rightServo");
        leftEncoder = hardwareMap.analogInput.get("leftEncoder");
        rightEncoder = hardwareMap.analogInput.get("rightEncoder");

        controller = new PIDFController(p, i, d, f);
    }

    public boolean updateServoArm() {

        lastLeftPos = currentLeftPos;
        lastRightPos = currentRightPos;

        leftPos = leftEncoder.getVoltage() / leftEncoder.getMaxVoltage() * 360;
        rightPos = rightEncoder.getVoltage() / rightEncoder.getMaxVoltage() * 360;

        currentLeftPos = leftPos;
        currentRightPos = rightPos;

        deltaLeftPos = lastLeftPos - currentLeftPos;
        deltaRightPos = lastRightPos - currentRightPos;

        if ((Math.abs(deltaLeftPos) > 90) || (Math.abs(deltaRightPos) > 90)) {
            if (rightServo.getPower() < 0) {
                universalRightPos -= 360;
            } else {
                universalRightPos += 360;
            }
            if (leftServo.getPower() < 0) {
                universalLeftPos -= 360;
            } else {
                universalLeftPos += 360;
            }
        }

        double rightPower = controller.calculate(targetRight, universalRightPos);
        double leftPower = controller.calculate(targetLeft, universalLeftPos);
//
        if (moving) {
            if (((targetLeft - range) <= universalLeftPos) && (universalLeftPos <= (targetLeft + range))) {
                leftPower = 0;
                rightMoving = false;
            }
            if (((targetRight - range) <= universalRightPos) && (universalRightPos <= (targetRight + range))) {
                rightPower = 0;
                leftMoving = false;
            }
        }

        if(rightMoving || leftMoving)
            moving = true;

        leftServo.setPower(leftPower);
        rightServo.setPower(rightPower);

        return moving;
    }

    public void runToProfile(double desiredRoll, double desiredPitch) {
        // Set your target angles based on desiredPitch and desiredRoll
        targetLeft = desiredRoll + desiredPitch;
        targetRight = desiredRoll - desiredPitch;

        double leftError = targetLeft - leftPos;
        double rightError = targetRight - rightPos;

        double leftPower = controller.calculate(leftError, 0);
        double rightPower = controller.calculate(rightError, 0);

//        double leftPower = controller.calculate(targetLeft, leftPos);
//        double rightPower = controller.calculate(targetRight, rightPos);

        leftServo.setPower(leftPower);
        rightServo.setPower(rightPower);

        moving = true;
    }

    public boolean isBusy() {
        return moving;
    }

    public double getLeftPosition() {
        return leftPos;  // Assuming you have a variable storing the left position
    }

    public double getRightPosition() {
        return rightPos;  // Assuming you have a variable storing the left position
    }

    public double getLeftTarget() {
        return targetLeft;  // Assuming you have a variable storing the left position
    }

    public double getRightTarget() {
        return targetRight;  // Assuming you have a variable storing the left position
    }

}