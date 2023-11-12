package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class horizontal {

    private Servo horizontalServo;
    private double horizontalPosition = 0;
    private double horizontalTarget = 0;
    private boolean moving = false;
    private double max = 0.63;
    private double min = 0.28;
    private double servoPosition = 0;
    private ServoController horizontalController;

    public horizontal(HardwareMap hardwareMap) {

        horizontalServo = hardwareMap.get(Servo.class, "extension");
        horizontalController = horizontalServo.getController();
      //  horizontalSwitch = hardwareMap.get(DigitalChannel.class, "horizontal switch");
        //horizontalEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontal"));

    }

//    public boolean updateHorizontal(){
//
//      //  horizontalPosition = horizontalEncoder.getCurrentPosition();
//      //  horizontalLimit = horizontalSwitch.getState();
//
////        if(horizontalLimit){
////            horizontalPosition = 0.35;
////        }
//
//        horizontalServo.setPosition(horizontalPosition);
//
//        if (moving){
//            if(((horizontalTarget - 0.01) <= horizontalPosition) && (horizontalPosition <= (horizontalTarget + 0.01))){
//                moving = false;
//            }
//        }
//
//        telemetry.addData("Horizontal Position", horizontalPosition);
//        telemetry.addData("Horizontal Target", horizontalTarget);
//        telemetry.addData("Horizontal Moving", moving);
//
//        return moving != false;
//    }

    public boolean extendHorizontal(double extensionDist){
//        moving = true;

//        double horizontalTargetPos = (Math.asin((extensionDist - 3) / 25.6)) * 0.66667 * 0.01 + 0.2;

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


        return moving != false;
    }

    public void setMin(double minPos)
    {
        min = minPos;
    }

    public boolean minDist(){
//        moving = true;

        horizontalServo.setPosition(min);

        return moving != false;
    }

    public boolean numDist(double num){
//        moving = true;

        horizontalServo.setPosition(num);

        return moving != false;
    }

    public boolean maxDist (){
//        moving = true;

        horizontalServo.setPosition(max);

        return moving != false;
    }

    public boolean setPos (double pos) {
        horizontalServo.setPosition(pos);

        return moving != false;
    }
    public double getPosition (){

        servoPosition = horizontalServo.getPosition();

        return servoPosition;
    }
    public double getPositionCM (){
        servoPosition = horizontalServo.getPosition();
        return (25.65 * Math.sin(.523596 - 2.61798*servoPosition));
    }
    public double move (double servoMove){

        servoPosition = horizontalServo.getPosition();
        servoPosition = servoPosition + servoMove;

        if (Double.isNaN(servoPosition))
        {
            servoPosition = max;
        }

        if (servoPosition>max) {
            servoPosition = max;
        }
        if (servoPosition<min) {
            servoPosition = min;
        }
        horizontalServo.setPosition(servoPosition);

        return servoPosition;
    }
    public void disable(){
        horizontalController.pwmDisable();
    }

}
