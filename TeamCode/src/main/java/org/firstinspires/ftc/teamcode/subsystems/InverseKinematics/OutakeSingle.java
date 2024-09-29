package org.firstinspires.ftc.teamcode.subsystems.InverseKinematics;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class OutakeSingle {

    private Arm1 arm1;
    private Elbow arm2;
    public ServoDiffyWrist wrist;
    private BackdropIVK ivk;
    private IVK myIVK;

    private double ServoArmDeg;
    private double MotorArmDeg;
    private double wristDeg;

    public static double extendedSitting = 95;
    public static double extendedIntaking = 0;

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

    public static double extIntWristPos = -125;
    public static double extIntRot = -192;
    public static double elseExtIntWristPos = 48;
    public static double elseExtIntRot = -250;

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

    public void IVKtoArmPoses(double arm1Pos, double arm2Pos, double wristPos, boolean useWrist, boolean extended, boolean intaking, double speed, boolean turned, double turnedValue){
        myAngle = angle;
        myIVK = (new IVK());

        ServoArmDeg = arm2Pos;
        MotorArmDeg = arm1Pos;

        double rotated = 0;

        if(turned)
            rotated = turnedValue;

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
                wristPos = extIntWristPos;
                rotated = extIntRot;
            } else {
                wristPos = elseExtIntWristPos;
                rotated = elseExtIntRot;
            }
        }

        ServoArmPos = -ServoArmDeg * 1000 / 90;
        MotorArmPos = MotorArmDeg * -1000 / 90;


        if(extended) {

            arm1.ArmToPos((int) Math.round(MotorArmPos), speed);
            arm2.ArmToPos((int) Math.round(ServoArmPos), speed);
            wrist.runToProfile(wristPos, rotated);

        }
        else {
            arm2.ArmToPos((int) Math.round(ServoArmPos), speed);
            arm1.ArmToPos((int) Math.round(MotorArmPos), speed*0.85);
            wrist.runToProfile(wristPos, rotated);
        }

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

