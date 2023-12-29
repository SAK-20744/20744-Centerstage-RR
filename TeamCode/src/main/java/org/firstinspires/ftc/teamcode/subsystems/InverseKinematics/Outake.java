package org.firstinspires.ftc.teamcode.subsystems.InverseKinematics;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outake {

    private Arm1 arm1;
    private Elbow arm2;
//    private ServoArm arm2;
    private Wrist wrist;
    private IVKMath ivk;

    private double ServoArmPos;
    private double MotorArmDeg;
    private double wristPos;

    public Outake(HardwareMap hardwareMap) {

        arm1 = (new Arm1(hardwareMap));
//        arm2 = new ServoArm(hardwareMap);
        arm2 = new Elbow(hardwareMap);
        wrist = (new Wrist(hardwareMap));
        ivk = (new IVKMath());

    }

    public void BackdropHeight(double length){

        ServoArmPos = ivk.q2backdrop(length);
        MotorArmDeg = ivk.q1Backdrop(length);
        wristPos = ivk.q3Backdrop(length);

        arm1.ArmToPos(((int)MotorArmDeg*(-1000/90)), 1);
        arm2.ArmToPos(((int)ServoArmPos*(-1000/90)), 1);
//        wrist.setPosDeg(wristPos);

    }

    public double getServoArmPos() {
        return ServoArmPos;
    }

    public double getMotorArmPos() {
        return MotorArmDeg;
    }

    public double getWristPos() {
        return wristPos;
    }
}

