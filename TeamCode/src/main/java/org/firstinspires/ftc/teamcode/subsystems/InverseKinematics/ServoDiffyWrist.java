package org.firstinspires.ftc.teamcode.subsystems.InverseKinematics;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ServoDiffyWrist {

    // Define your CRServo objects
    private Servo leftServo;
    private Servo rightServo;

    private double leftPos, rightPos, leftServoPos, rightServoPos;

    public static double pitchOffset = 270, rollOffset = 95;

    public ServoDiffyWrist(HardwareMap hardwareMap) {

        leftServo = hardwareMap.servo.get("rightServo");
        rightServo = hardwareMap.servo.get("leftServo");
    }

    public void runToProfile(double desiredRoll, double desiredPitch) {

        desiredPitch = desiredPitch + pitchOffset;
        desiredRoll = -desiredRoll - rollOffset;
 
        leftPos =  desiredPitch + desiredRoll;
        rightPos = desiredPitch - desiredRoll;

        leftServoPos = (leftPos) / 332.725;
        rightServoPos = (rightPos) / 332.725;

        leftServo.setPosition(leftServoPos);
        rightServo.setPosition(rightServoPos);

    }

    public double getLeftPos(){return leftPos;}
    public double getRightPos(){return rightPos;}
    public double getLeftServoPos(){return leftServoPos;}
    public double getRightServoPos(){return rightServoPos;}

}