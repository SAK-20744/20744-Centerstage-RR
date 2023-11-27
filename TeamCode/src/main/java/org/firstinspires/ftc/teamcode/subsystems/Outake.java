package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outake {

    private Arm1 arm1;
    private ServoArm arm2;
    private Wrist wrist;
    private IVK ivk;

    public Outake(HardwareMap hardwareMap) {
        ivk = (new IVK());
        arm1 = (new Arm1(hardwareMap));
        arm2 = (new ServoArm(hardwareMap));
        wrist = (new Wrist(hardwareMap));
    }

    public void BackdropHeight(double length){
        double ServoArmPos = ivk.q2backdrop(length);
        double MotorArmPos = ivk.q1Backdrop(length);
        double wristPos = ivk.q3Backdrop(length);

        arm1.ArmToDeg(MotorArmPos, 1);
        arm2.runToProfile(ServoArmPos);
        wrist.setPosDeg(wristPos);

    }

}

