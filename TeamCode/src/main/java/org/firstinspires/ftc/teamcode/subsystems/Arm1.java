package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Base64;

public class Arm1 {

    private DcMotor left_Arm1;
    private DcMotor right_Arm1;

    private double leftArm1Position = 0;
    private double rightArm1Position = 0;

    private double leftArm1Power = 0;
    private double rightArm1Power = 0;

    public Arm1(HardwareMap hardwareMap) {
        left_Arm1 = hardwareMap.get(DcMotor.class, "left_lift");
        right_Arm1 = hardwareMap.get(DcMotor.class, "right_lift");

        left_Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void updateArm1(){
        leftArm1Position = left_Arm1.getCurrentPosition();
        rightArm1Position = right_Arm1.getCurrentPosition();

        left_Arm1.setPower(leftArm1Power);
        right_Arm1.setPower(rightArm1Power);
    }
    
    public void resetArm1(){
        left_Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    public void move(double Arm1SetPower){

        leftArm1Power = -Arm1SetPower;
        rightArm1Power = Arm1SetPower;
        left_Arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_Arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void ArmToPos(int pos){

        left_Arm1.setTargetPosition(-pos);
        right_Arm1.setTargetPosition(pos);

        left_Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void cancel()
    {
        left_Arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_Arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_Arm1.setPower(0);
        right_Arm1.setPower(0);
    }

    public double getLeftArm1Position() {
        leftArm1Position = left_Arm1.getCurrentPosition();
        return leftArm1Position;
    }
    
    public double getRightArm1Power(){
        rightArm1Position = right_Arm1.getCurrentPosition();
        return rightArm1Position;
    }


}
