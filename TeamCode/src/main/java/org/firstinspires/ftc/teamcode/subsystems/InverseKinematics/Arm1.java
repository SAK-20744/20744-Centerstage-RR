package org.firstinspires.ftc.teamcode.subsystems.InverseKinematics;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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

    private double degreesToTicksFactor = -1000/90;

    private boolean moving;

    public Arm1(HardwareMap hardwareMap) {
        left_Arm1 = hardwareMap.get(DcMotor.class, "left_lift");
        right_Arm1 = hardwareMap.get(DcMotor.class, "right_lift");

        left_Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public boolean updateArm1(){
        leftArm1Position = left_Arm1.getCurrentPosition();
        rightArm1Position = right_Arm1.getCurrentPosition();

        if (moving){
            if(!left_Arm1.isBusy()){
                moving = false;
            }
        }

        left_Arm1.setPower(leftArm1Power);
        right_Arm1.setPower(rightArm1Power);

        return rightArm1Power != 0;
    }

    public double getDegreesToTicksFactor(double degrees){
        return degreesToTicksFactor * degrees;
    }

    public void resetArm1(){
        left_Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
//
//    public void move(double Arm1SetPower){
//
//        leftArm1Power = -Arm1SetPower;
//        rightArm1Power = Arm1SetPower;
//        left_Arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        right_Arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }

    public boolean move(double Arm1SetPower){
        if (Arm1SetPower == 0){
            moving = false;
        }
        if(!moving) {
            leftArm1Power = -Arm1SetPower;
            rightArm1Power = Arm1SetPower;
            left_Arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right_Arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        return Arm1SetPower != 0;
    }

    public boolean ArmToPos(int pos, double power){
        moving = true;

        left_Arm1.setTargetPosition(pos);
        right_Arm1.setTargetPosition(pos);

        left_Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(left_Arm1.getCurrentPosition()>=pos) {
            left_Arm1.setPower(power);
            right_Arm1.setPower(power);
        }
        else {
            left_Arm1.setPower(-power);
            right_Arm1.setPower(-power);
        }
        return power != 0;
    }

    public void ArmToDeg(double degrees, double power){

        int pos = (int)((degrees)*-1000/90);

        left_Arm1.setTargetPosition(pos);
        right_Arm1.setTargetPosition(pos);

        left_Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(left_Arm1.getCurrentPosition()>pos) {
            left_Arm1.setPower(power);
            right_Arm1.setPower(power);
        }
        else {
            left_Arm1.setPower(-power);
            right_Arm1.setPower(-power);
        }

    }

    public void cancel()
    {
        left_Arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_Arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_Arm1.setPower(0);
        right_Arm1.setPower(0);
        moving = false;
    }

    public double getLeftArm1Position() {
        leftArm1Position = left_Arm1.getCurrentPosition();
        return leftArm1Position;
    }
    
    public double getRightArm1Position(){
        rightArm1Position = right_Arm1.getCurrentPosition();
        return rightArm1Position;
    }

    public boolean isBusy(){
        if(moving) {
            return true;
        }
        return false;
    }

}
