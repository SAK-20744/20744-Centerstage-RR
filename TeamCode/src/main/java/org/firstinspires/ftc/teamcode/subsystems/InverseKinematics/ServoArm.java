package org.firstinspires.ftc.teamcode.subsystems.InverseKinematics;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDFController;

public class ServoArm {

    private DcMotor left_lift;
    private DcMotor right_lift;

    private CRServo servoArmServo;
    private CRServo leftArm;
    private CRServo rightArm;
    private AnalogInput leftAnalogInput;
    private AnalogInput rightAnalogInput;

    private PIDFController controller;

    private double currentArmPos = 0;
    private double lastArmPos = 0;
    private double deltaArmPos;

    private double universalArmPos = 0;
    private double uncorrectedArmPos = 0;
    private double correctedArmPos = 0;

    private double leftPos;
    private double rightPos;
    
    private double p = 0.0167;
    private double i = 0.008;
    private double d = 0.00014;
    private double f = 0.00065;

    private double servoArmTarget;
    private double armPower;
    private boolean moving;

    public ServoArm(HardwareMap hardwareMap) {

        left_lift = hardwareMap.get(DcMotor.class, "left_lift");
        right_lift = hardwareMap.get(DcMotor.class, "right_lift");
        leftAnalogInput = hardwareMap.get(AnalogInput.class, "left");
        rightAnalogInput = hardwareMap.get(AnalogInput.class, "right");
        leftArm = hardwareMap.get(CRServo.class, "leftArm");
        rightArm = hardwareMap.get(CRServo.class, "rightArm");

        controller = new PIDFController(p, i, d, f);
    }
    
    public boolean updateServoArm()
    {
        lastArmPos = currentArmPos;

        leftPos = leftAnalogInput.getVoltage() / leftAnalogInput.getMaxVoltage() * 360;
        rightPos = rightAnalogInput.getVoltage() / rightAnalogInput.getMaxVoltage() * 360;

        currentArmPos = rightPos;
        deltaArmPos = lastArmPos - currentArmPos;

        if (Math.abs(deltaArmPos)>90)
        {
            if(rightArm.getPower()<0)
            {
                universalArmPos-=360;
            }
            else
            {
                universalArmPos+=360;
            }
        }
        uncorrectedArmPos = universalArmPos + rightPos;
        correctedArmPos = -1*uncorrectedArmPos/3;

        double armPower = controller.calculate(servoArmTarget, correctedArmPos);

        if (moving){
            if(((servoArmTarget - 5) <= correctedArmPos) && (correctedArmPos <= (servoArmTarget + 5))){
                armPower = 0;
                moving = false;
            }
        }

        rightArm.setPower(armPower);
        leftArm.setPower(-armPower);

        return armPower != 0;
    }

    public double getLocation()
    {
        return correctedArmPos;
    }

    public void runToProfile(double target)
    {
        moving = true;
        servoArmTarget = target;
        double armPower = controller.calculate(target, correctedArmPos);
        rightArm.setPower(armPower);
        leftArm.setPower(-armPower);
    }

    public boolean isBusy(){
        if(moving) {
            return true;
        }
        return false;
    }
}
