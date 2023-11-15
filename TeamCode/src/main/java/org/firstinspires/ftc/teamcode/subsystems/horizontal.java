package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class horizontal {

    private Servo horizontalServo;
    private double horizontalPosition = 0;
    private double horizontalTarget = 0;
    private boolean moving = false;
    private double max = 1;
    private double min = 0;
    private double servoPosition = 0;
    private ServoController horizontalController;

    public horizontal(HardwareMap hardwareMap) {
        horizontalServo = hardwareMap.get(Servo.class, "extension");
    }

    public void updatehorizontal(){
        telemetry.addData("horizontal Position", horizontalPosition);
        telemetry.addData("horizontal Target", horizontalTarget);
    }

    public void extendhorizontal(double extensionDist){
        double horizontalTargetPos = (Math.asin((extensionDist) / 25.6) / Math.PI * 180) * 0.66667 * 0.01 + 0.2;

        if (horizontalTargetPos>=max) {
            horizontalTargetPos = max;
        }
        if (horizontalTargetPos<=min) {
            horizontalTargetPos = min;
        }

        if (Double.isNaN(horizontalTargetPos))
        {
          if (extensionDist>25)
          {
              horizontalTargetPos = max;
          }
          else
          {
              horizontalTargetPos = min;
          }

        }

        horizontalServo.setPosition(horizontalTargetPos);
        servoPosition = horizontalTargetPos;

    }

    public void minDist(){
        horizontalServo.setPosition(min);
    }

    public void numDist(double num){
        horizontalServo.setPosition(num);
    }

    public void maxDist (){
        horizontalServo.setPosition(max);
    }

    public void setPos (double pos) {
        horizontalServo.setPosition(pos);
    }
    public double getPosition (){
        servoPosition = horizontalServo.getPosition();
        return servoPosition;
    }

    public double move (double servoMove){

        servoPosition = horizontalServo.getPosition();
        servoPosition = servoPosition + servoMove;

        if (Double.isNaN(servoPosition))
            servoPosition = max;

        if (servoPosition>max)
            servoPosition = max;

        if (servoPosition<min)
            servoPosition = min;

        horizontalServo.setPosition(servoPosition);

        return servoPosition;
    }
    public void disable(){
        horizontalController.pwmDisable();
    }

}
