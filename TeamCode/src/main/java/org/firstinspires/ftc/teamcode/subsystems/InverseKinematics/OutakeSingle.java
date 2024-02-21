package org.firstinspires.ftc.teamcode.subsystems.InverseKinematics;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class OutakeSingle {

    private Arm1 arm1;
    private Elbow arm2;
    public ServoDiffyWrist wrist;
    private BackdropIVK ivk;
    private IVK myIVK;

    private double ServoArmDeg;
    private double MotorArmDeg;
    private double wristDeg;

    private double ServoArmPos;
    private double MotorArmPos;
    private double wristPos;

    private double speed;

    private double myAngle;

    private double angleArmDistOffset = 2.4;
    private double angle = 60;
    private double usedAngle = angle - angleArmDistOffset;
    private double max = 33.5;
    private double offset6Max = 30.095;
    private double offset10Max = 27.361;

    public OutakeSingle(HardwareMap hardwareMap) {
        arm1 = (new Arm1(hardwareMap));
        arm2 = new Elbow(hardwareMap);
        wrist = (new ServoDiffyWrist(hardwareMap));
    }
//
//    public void BackdropHeight(double length, boolean intaking){
//
//        myAngle = angle;
//        ivk = (new BackdropIVK(myAngle));
//
//        if(length>max)
//            length = max;
//        if(length<0)
//            length = 0;
//
//        if(length>0) {
//            ServoArmDeg = ivk.q2(length);
//            MotorArmDeg = ivk.q1(length);
//            wristDeg = ivk.q3(length);
//        }
//        else {
//            ServoArmDeg = -28;
//            MotorArmDeg = 180;
//            if(intaking)
//                wristDeg = 0;
//            else
//                wristDeg = 90;
//        }
//            ServoArmPos = -ServoArmDeg * 1000 / 90;
//            MotorArmPos = MotorArmDeg * -1000 / 90;
//            wristPos = (6711.53 * Math.pow(wristDeg, -2.09156))-0.109251;
//
//        arm1.ArmToPos((int)Math.round(MotorArmPos), 1);
//        arm2.ArmToPos((int)Math.round(ServoArmPos), 1);
//        wrist.setPos(wristPos);
//
//    }
//
//    public void IVKtoXY(double X, double Y, boolean backdrop, boolean intaking, double speed){
//        myAngle = angle;
//        myIVK = (new IVK());
//
//        ServoArmDeg = myIVK.q2(X,Y);
//        MotorArmDeg = myIVK.q1(X,Y);
//        wristDeg = myIVK.q3(X,Y);
//
//        if(backdrop) {
//            wristPos = (wristDeg / 190);
//        }
//        else {
//            if (intaking)
//                wristDeg = 0;
//            else
//                wristDeg = 95;
//        }
//
//        ServoArmPos = -ServoArmDeg * 1000 / 90;
//        MotorArmPos = MotorArmDeg * -1000 / 90;
//
//        arm1.ArmToPos((int)Math.round(MotorArmPos), speed);
//        arm2.ArmToPos((int)Math.round(ServoArmPos), speed);
//        wrist.setPos(wristPos);
//    };

    public void IVKtoArmPoses(double arm1Pos, double arm2Pos, double wristPos, boolean useWrist, boolean extended, boolean intaking, double speed){
        myAngle = angle;
        myIVK = (new IVK());

        ServoArmDeg = arm2Pos;
        MotorArmDeg = arm1Pos;

        double rotated = 0;

        if(useWrist) {
            if (intaking) {
//                wristDeg = 0;
                wristPos = 4;
//                wristPos = (wristDeg / 190);
            } else {
//                wristDeg = 95;
                wristPos = -125;
//                wristPos = (wristDeg / 190);
            }
        }

        if(extended) {
            if (intaking) {
                wristPos = -125;
                rotated = -235;
            } else {
                wristPos = 50;
                rotated = -225;
            }
        }

        ServoArmPos = -ServoArmDeg * 1000 / 90;
        MotorArmPos = MotorArmDeg * -1000 / 90;

        arm1.ArmToPos((int)Math.round(MotorArmPos), speed);
        arm2.ArmToPos((int)Math.round(ServoArmPos), speed);
        wrist.runToProfile(wristPos, rotated);
//        while(wrist.isBusy()) {
//            wrist.updateDiffy();
//            telemetry.addData("leftPos: ", wrist.getLeftPosition());
//            telemetry.addData("rightPos: ", wrist.getRightPosition());
//            telemetry.addData("leftCorrectedPos: ", wrist.getCorrectedLeftPos());
//            telemetry.addData("rightCorrectedPos: ", wrist.getCorrectedRightPos());
//            telemetry.addData("leftTarget: ", wrist.getLeftTarget());
//            telemetry.addData("rightTarget: ", wrist.getRightTarget());
//            telemetry.addData("Moving?: ", wrist.isBusy());
//            telemetry.update();
//        }
//        wrist.updateDiffy();

    };

//    public void BackdropHeightOffset10(double length, boolean intaking, boolean hang){
//
//        myAngle = angle;
//        ivk = (new BackdropIVK(myAngle));
//
//        if(length>offset10Max)
//            length = offset10Max;
//        if(length<4)
//            length = 4;
//
//        if(length>4 && length<8) {
//            length = 8.1;
//            speed = 0.5;
//        }
////
////        if(length == 8.1)
////            speed = 0.5;
//
////        if(length > 0 && length < 8.5){
////            length = 8;
////        }
//
//        if(hang) {
//            speed = 1;
//            length = 6;
//        }
//
//        if(length>0) {
//            ServoArmDeg = ivk.q2(length);
//            MotorArmDeg = ivk.q1(length);
//            wristDeg = ivk.q3(length);
//        }
//        if(length > 8) {
//            speed = 1;
//        }
////        if(length == 4){
////            ServoArmDeg = ;
////            MotorArmDeg = 90;
////            wristDeg = 100;
////        }
//        else {
//            ServoArmDeg = -32;
//            MotorArmDeg = 186;
//            if(intaking)
//                wristDeg = 0;
//            else
//                wristDeg = 95;
//        }
//        ServoArmPos = ServoArmDeg * -1000 / 90;
//        MotorArmPos = MotorArmDeg * -1000 / 90;
//        wristPos = (wristDeg/190);
//
//        arm1.ArmToPos((int)Math.round(MotorArmPos), speed);
//        arm2.ArmToPos((int)Math.round(ServoArmPos), speed);
//        wrist.setPos(wristPos);
//
//
//    }

    public double getServoArmDeg() {
        return ServoArmDeg;
    }
    public double getMotorArmDeg() {
        return MotorArmDeg;
    }
    public double getWristDeg() {
        return wristDeg;
    }

    public double getServoArmPos() {
        return ServoArmPos;
    }
    public double getMotorArmPos() {
        return MotorArmPos;
    }
    public double getWristPos() {
        return wristPos;
    }


//    public double getA1(double length) {
//        return ivk.getA1(length);
//    }
//
//    public double getQ1(double length) {
//        return ivk.getQ1(length);
//    }
}

