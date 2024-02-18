package org.firstinspires.ftc.teamcode.subsystems.InverseKinematics;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
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

    private PIDFController leftController;
    private PIDFController rightController;

    private double leftPos;
    private double rightPos;

    public static double targetLeft;
    public static double targetRight;

    private double correctedLeftPosition, correctedRightPosition;

    private boolean moving, rightMoving, leftMoving;

    public static double pL = 0.0086;
    public static double iL = 0.02  ;
    public static double dL = 0;
    public static double fL = 0;

    public static double pR = 0.01;
    public static double iR = 0.05;
    public static double dR = 0;
    public static double fR = 0.015;

    public static double range = 5;
    public static double idealRange = 1.5;
    public static double offset = -13.5;

    private double lastLeftPos, lastRightPos, currentLeftPos, currentRightPos, deltaLeftPos, deltaRightPos, universalRightPos, universalLeftPos;

    public DiffyWrist(HardwareMap hardwareMap) {

        leftServo = hardwareMap.crservo.get("rightServo");
        rightServo = hardwareMap.crservo.get("leftServo");
        leftEncoder = hardwareMap.analogInput.get("rightEncoder");
        rightEncoder = hardwareMap.analogInput.get("leftEncoder");

        leftController = new PIDFController(pL, iL, dL, fL);
        rightController = new PIDFController(pR, iR, dR, fR);
    }

    public boolean updateDiffy() {

        lastLeftPos = currentLeftPos;
        lastRightPos = currentRightPos;

        leftPos = leftEncoder.getVoltage() / leftEncoder.getMaxVoltage() * 360;
        rightPos = rightEncoder.getVoltage() / rightEncoder.getMaxVoltage() * 360;

        currentLeftPos = leftPos;
        currentRightPos = rightPos;

        deltaLeftPos = lastLeftPos - currentLeftPos;
        deltaRightPos = lastRightPos - currentRightPos;

//        if ((Math.abs(deltaLeftPos) > 180) || (Math.abs(deltaRightPos) > 180)) {
//            if (rightServo.getPower() < 0) {
//                universalRightPos -= 360;
//            } else {
//                universalRightPos += 360;
//            }
//            if (leftServo.getPower() < 0) {
//                universalLeftPos -= 360;
//            } else {
//                universalLeftPos += 360;
//            }
//        }

        if ((Math.abs(deltaLeftPos) > 180)){
            if (leftServo.getPower() < 0) {
                universalLeftPos += 360;
            } else {
                universalLeftPos -= 360;
            }
        }

        if ((Math.abs(deltaRightPos) > 180)){
            if (rightServo.getPower() < 0) {
                universalRightPos += 360;
            } else {
                universalRightPos -= 360;
            }
        }

//        if ((Math.abs(deltaLeftPos) > 180)){
//            if (deltaLeftPos > 0) {
//                universalLeftPos = 360 - universalLeftPos;
//            } else {
//                universalLeftPos = 360 + universalLeftPos;
//            }
//        }
//
//        if ((Math.abs(deltaRightPos) > 180)){
//            if (deltaRightPos > 0) {
//                universalRightPos = 360 - universalRightPos;
//            } else {
//                universalRightPos = 360 + universalRightPos;
//            }
//        }

        correctedLeftPosition = universalLeftPos + leftPos;
        correctedRightPosition = universalRightPos + rightPos;

        double leftError = targetLeft - correctedLeftPosition;
        double rightError = targetRight - correctedRightPosition;
//
        double leftPower = leftController.calculate(leftError, 0);
        double rightPower = rightController.calculate(rightError, 0);

//        double rightPower = controller.calculate(correctedRightPosition, targetRight);
//        double leftPower = controller.calculate(correctedLeftPosition, targetLeft);
//
        if (moving) {
            if(leftMoving) {
                if (((targetLeft - idealRange) <= correctedLeftPosition) && (correctedLeftPosition <= (targetLeft + idealRange))) {
                    leftPower = 0;
                    leftMoving = false;
                }
                else if (((targetLeft - range) <= correctedLeftPosition) && (correctedLeftPosition <= (targetLeft + range))) {
                    leftMoving = false;
                }
                else{
                    leftMoving = true;
                }
            }
            if(rightMoving) {
                if (((targetRight - idealRange) <= correctedRightPosition) && (correctedRightPosition <= (targetRight + idealRange))) {
                    rightPower = 0;
                    rightMoving = false;
                }
                else if (((targetRight - range) <= correctedRightPosition) && (correctedRightPosition <= (targetRight + range))) {
                    rightMoving = false;
                }
                else {
                    rightMoving = true;
                }
            }
        }
        else {
            rightPower = 0;
            leftPower = 0;
            moving = false;
            rightMoving = false;
            leftMoving = false;
        }

        if(rightMoving || leftMoving)
            moving = true;
        else
            moving = false;

        rightServo.setPower(rightPower);
        leftServo.setPower(leftPower);

        return moving;

    }

//    public void updateServoArm() {
//
//        leftPos = leftEncoder.getVoltage() / leftEncoder.getMaxVoltage() * 360;
//        rightPos = rightEncoder.getVoltage() / rightEncoder.getMaxVoltage() * 360;
//
//        double rightPower = controller.calculate(targetRight, leftPos);
//        double leftPower = controller.calculate(targetLeft, rightPos);
//
//        leftServo.setPower(leftPower);
//        rightServo.setPower(rightPower);
//
//    }

    public void runToProfile(double desiredRoll, double desiredPitch) {
        // Set your target angles based on desiredPitch and desiredRoll

        desiredPitch = -(desiredPitch - offset);
        desiredRoll = -(desiredRoll);

        targetLeft =  desiredPitch + desiredRoll;
        targetRight = desiredPitch - desiredRoll;

//        double leftError = targetLeft - leftPos;
//        double rightError = targetRight - rightPos;
//
//        double leftPower = controller.calculate(leftError, 0);
//        double rightPower = controller.calculate(rightError, 0);

//        leftServo.setPower(leftPower);
//        rightServo.setPower(rightPower);

        moving = true;
        leftMoving = true;
        rightMoving = true;
    }

    public void movePitch(double desiredRoll){

        targetLeft += desiredRoll;
        targetRight -= desiredRoll;

        moving = true;
        leftMoving = true;
        rightMoving = true;

    }

    public void moveRoll(double desiredRoll){

        targetLeft += desiredPitch;
        targetRight = desiredPitch;

        moving = true;
        leftMoving = true;
        rightMoving = true;

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

    public double getCorrectedRightPos() {
        return correctedRightPosition;  // Assuming you have a variable storing the left position
    }

    public double getCorrectedLeftPos() {
        return correctedLeftPosition;  // Assuming you have a variable storing the left position
    }

}