package org.firstinspires.ftc.teamcode.subsystems.InverseKinematics;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ActiveIntakeV1 {

    private CRServo intakeServo;
    private Servo wristServo;

    private double slowSpeed = 0.15;
    private double fullSpeed = -1;

    private double rest = 0;
    private double backdrop = 0;
    private double intaking = 0;
    private double stack = 0;

    public ActiveIntakeV1(HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(CRServo.class, "intake");
        wristServo = hardwareMap.get(Servo.class, "wrist");
    }

    public void in(){
        intakeServo.setPower(fullSpeed);
    }

    public void out(){
        intakeServo.setPower(slowSpeed);
    }

    public void wristToPos(double pos){
        wristServo.setPosition(pos);
    }

    public void wristRest(){
        wristServo.setPosition(rest);
    }

    public void wristBackdrop(){
        wristServo.setPosition(backdrop);
    }

    public void wristIntake(){
        wristServo.setPosition(intaking);
    }

    public void wristStack(){
        wristServo.setPosition(stack);
    }

}
