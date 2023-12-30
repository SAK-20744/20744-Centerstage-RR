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
    
    
    public Outake(HardwareMap hardwareMap) {

        arm1 = (new Arm1(hardwareMap));
        arm2 = new Elbow(hardwareMap);
        wrist = (new Wrist(hardwareMap));
        ivk = (new IVKMath());

    }

    public void BackdropHeight(double length){

        if(length>0) {
            ServoArmDeg = ivk.q2(length);
            MotorArmDeg = ivk.q1(length);
            wristDeg = ivk.q3(length);
        }
        else {
            ServoArmDeg = 0;
            MotorArmDeg = 180;
            wristDeg = 60;
        }
            ServoArmPos = -ServoArmDeg * 1000 / 90;
            MotorArmPos = MotorArmDeg * -1000 / 90;
            wristPos = (wristDeg/190);


        arm1.ArmToPos((int)Math.round(MotorArmPos), 1);
        arm2.ArmToPos((int)Math.round(ServoArmPos), 1);
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

