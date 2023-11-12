package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class intake {

    private Servo intakeServo;
    private DigitalChannel intakeSwitch;
    private boolean intakeLimit = false;
    private boolean moving = false;
    private final double MAX = 0.5;
    private final double MIN = 0.2;



    public intake(HardwareMap hardwareMap) {

        intakeServo = hardwareMap.get(Servo.class, "intake");
        intakeSwitch = hardwareMap.get(DigitalChannel.class, "intake switch");
        //horizontalEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontal"));

    }

   public void updateIntake() {

       intakeLimit = intakeSwitch.getState();

   }

   public boolean intakeState()
   {
       intakeLimit = intakeSwitch.getState();
       return intakeLimit;

   }


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

//    public boolean extendHorizontal(double intakeDist){
////        moving = true;
//
////        double horizontalTargetPos = (Math.asin((extensionDist - 3) / 25.6)) * 0.66667 * 0.01 + 0.2;
//
////        double horizontalTargetPos = (Math.asin((extensionDist - 3) / 25.6) / Math.PI * 180) * 0.66667 * 0.01 + 0.2;
//        if (intakeDist>0.6) {
//            intakeDist = 0.6;
//        }
//        if (intakeDist<0.2) {
//            intakeDist = 0.2;
//        }
//        intakeServo.setPosition(intakeDist);
//
//
//        return moving != false;
//    }

    public void release(){
//        moving = true;

        intakeServo.setPosition(MIN);

    }

    public void grab(){
//        moving = true;

        intakeServo.setPosition(MAX);

    }

//    public boolean isBusy(){
//        if(!moving) {
//            return true;
//        }
//        return false;
//    }

}
