package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class wrist {

    private Servo wristServo;
    private double wristPosition = 0;
    private double wristTarget = 0;
    private boolean moving = false;
    private double max = 1;
    private double min = 0;
    private double servoPosition = 0;
    private ServoController wristController;

    public wrist(HardwareMap hardwareMap) {
        wristServo = hardwareMap.get(Servo.class, "extension");
    }

    public void updatewrist(){
        telemetry.addData("wrist Position", wristPosition);
        telemetry.addData("wrist Target", wristTarget);
    }

    public void extendwrist(double extensionDist){
        double wristTargetPos = (Math.asin((extensionDist) / 25.6) / Math.PI * 180) * 0.66667 * 0.01 + 0.2;

        if (wristTargetPos>=max) {
            wristTargetPos = max;
        }
        if (wristTargetPos<=min) {
            wristTargetPos = min;
        }

        if (Double.isNaN(wristTargetPos))
        {
          if (extensionDist>25)
          {
              wristTargetPos = max;
          }
          else
          {
              wristTargetPos = min;
          }

        }

        wristServo.setPosition(wristTargetPos);
        servoPosition = wristTargetPos;

    }

    public void minDist(){
        wristServo.setPosition(min);
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
    public double getPosition (){
        servoPosition = wristServo.getPosition();
        return servoPosition;
    }
//
//    public double move (double servoMove){
//
//        servoPosition = wristServo.getPosition();
//        servoPosition = servoPosition + servoMove;
//
//        if (Double.isNaN(servoPosition))
//            servoPosition = max;
//
//        if (servoPosition>max)
//            servoPosition = max;
//
//        if (servoPosition<min)
//            servoPosition = min;
//
//        wristServo.setPosition(servoPosition);
//
//        return servoPosition;
//    }

    public void disable(){
        wristController.pwmDisable();
    }

}
