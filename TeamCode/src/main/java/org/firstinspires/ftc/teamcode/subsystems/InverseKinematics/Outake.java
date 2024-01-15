package org.firstinspires.ftc.teamcode.subsystems.InverseKinematics;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outake {

    private Arm1 arm1;
    private Elbow arm2;
    private Wrist wrist;
    private IVKMath ivk;

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
    
    public Outake(HardwareMap hardwareMap) {
        arm1 = (new Arm1(hardwareMap));
        arm2 = new Elbow(hardwareMap);
        wrist = (new Wrist(hardwareMap));
    }

    public void BackdropHeight(double length, boolean intaking){

        myAngle = angle;
        ivk = (new IVKMath(myAngle));

        if(length>max)
            length = max;
        if(length<0)
            length = 0;

        if(length>0) {
            ServoArmDeg = ivk.q2(length);
            MotorArmDeg = ivk.q1(length);
            wristDeg = ivk.q3(length);
        }
        else {
            ServoArmDeg = -28;
            MotorArmDeg = 180;
            if(intaking)
                wristDeg = 0;
            else
                wristDeg = 90;
        }
            ServoArmPos = -ServoArmDeg * 1000 / 90;
            MotorArmPos = MotorArmDeg * -1000 / 90;
            wristPos = (wristDeg/190);

        arm1.ArmToPos((int)Math.round(MotorArmPos), 1);
        arm2.ArmToPos((int)Math.round(ServoArmPos), 1);
        wrist.setPos(wristPos);

    }

    public void BackdropHeightOffset10(double length, boolean intaking, boolean hang){

        myAngle = angle;
        ivk = (new IVKMath(myAngle));

        if(length>offset10Max)
            length = offset10Max;
        if(length<0)
            length = 0;
//        if(length>0 && length<4)
//            length = 4;
//        if(length>4 && length<8)
//            length = 8;
        if(length>0 && length<8) {
            length = 8;
            speed = 0.5;
        }

        if(length < 5.5){
            length = 5.5;
        }

        if(length>0) {
            ServoArmDeg = ivk.q2(length);
            MotorArmDeg = ivk.q1(length);
            wristDeg = ivk.q3(length);
        }
        if(length > 8) {
            speed = 1;
        }
//        if(length == 4){
//            ServoArmDeg = ;
//            MotorArmDeg = 90;
//            wristDeg = 100;
//        }
        else {
            ServoArmDeg = -32;
            MotorArmDeg = 186;
            if(intaking)
                wristDeg = 0;
            else
                wristDeg = 95;
        }
        ServoArmPos = -ServoArmDeg * 1000 / 90;
        MotorArmPos = MotorArmDeg * -1000 / 90;
//        wristPos = (wristDeg/190);
//        wristPos = (6711.53 * Math.pow(wristDeg, -2.09156))-0.109251;

        if(hang)
            speed = 1;

        arm1.ArmToPos((int)Math.round(MotorArmPos), speed);
        arm2.ArmToPos((int)Math.round(ServoArmPos), speed);
        wrist.setPos(wristPos);

    }

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

