package org.firstinspires.ftc.teamcode.subsystems.InverseKinematics;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class Wrist {

    private Servo wristServo;
    private double wristPosition = 0;
    private double wristTarget = 0;
    private boolean moving = false;
    private double max = 1;
    private double min = 0;
    private double servoPosition = 0;

    private double driving = 0.63;
    private double intaking = 0.24;

    private ServoController wristController;

    public Wrist(HardwareMap hardwareMap) {
        wristServo = hardwareMap.get(Servo.class, "wrist");
    }

    public void updatewrist(){
        telemetry.addData("wrist Position", wristPosition);
        telemetry.addData("wrist Target", wristTarget);
    }

    public void minDist(){
        wristServo.setPosition(min);
    }

    public void driving(){
        wristServo.setPosition(0.63);
    }

    public void intaking(){
        wristServo.setPosition(0.24);
    }

    public void numDist(double num){
        wristServo.setPosition(num);
    }

    public void maxDist (){
        wristServo.setPosition(max);
    }

    public void setPos (double pos) {
        wristServo.setPosition(pos);
    }

    public void setPosDeg (double deg) {
        wristServo.setPosition((deg-10)/180);
    }

    public double getPosition (){
        servoPosition = wristServo.getPosition();
        return servoPosition;
    }

    public void disable(){
        wristController.pwmDisable();
    }

}
