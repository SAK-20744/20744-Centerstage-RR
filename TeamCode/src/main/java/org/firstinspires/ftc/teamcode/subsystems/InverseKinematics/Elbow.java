package org.firstinspires.ftc.teamcode.subsystems.InverseKinematics;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config

public class Elbow {

    private DcMotor elbow;
    private DcMotor elbow2;

    private double elbowPosition;
    private double elbow2Position;

    public static double range = 10;

    private double target;

    private double elbowPower = 0;
    private double elbowPower2 = 0;

    private double degreesToTicksFactor = -1000/90;

    private boolean moving;

    public Elbow(HardwareMap hardwareMap) {
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow2 = hardwareMap.get(DcMotor.class, "elbow2");
        elbow2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public boolean updateElbow(){
        elbowPosition = elbow.getCurrentPosition();
        elbow2Position = elbow2.getCurrentPosition();

        if (moving){
            if(!elbow.isBusy()){
                moving = false;
            }
        }

        elbow.setPower(elbowPower);
        elbow2.setPower(elbowPower2);

        return elbowPower != 0;
    }

    public void shutOff(){

        if((elbowPosition-range)> target && target>(elbowPosition+range))
        {
            elbowPower = 0;
        }

        if(elbow.getCurrentPosition() >= target) {
            elbow.setPower(elbowPower);
        }
        else {
            elbow.setPower(-elbowPower);
        }

    }

    public double getDegreesToTicksFactor(double degrees){
        return degreesToTicksFactor * degrees;
    }

    public void resetArm1(){
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

//    public boolean move(double Arm2SetPower){
//        if (Arm2SetPower == 0){
//            moving = false;
//        }
//        if(!moving) {
//            elbowPower = Arm2SetPower;
//            elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            elbowPower = Arm2SetPower;
//            elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//        return Arm1SetPower != 0;
//    }

//    public boolean ArmToPos(int pos, double power){
//        elbowPower = power;
//        moving = true;
//        target = pos;
//        elbow.setTargetPosition(pos);
//        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        if(elbow.getCurrentPosition()>=pos) {
//            elbow.setPower(power);
//        }
//        else {
//            elbow.setPower(-power);
//        }
//        return power != 0;
//    }

    public boolean ArmToPos(int pos, double power){
        moving = true;

        elbow.setTargetPosition(pos);
        elbow2.setTargetPosition(pos);

        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(elbow.getCurrentPosition()>=pos) {
            elbow.setPower(power);
            elbow2.setPower(power);
        }
        else {
            elbow.setPower(-power);
            elbow2.setPower(-power);
        }
        return power != 0;
    }

    public void ArmToDeg(double degrees, double power){

        int pos = (int)((degrees)*-1000/90);

        elbow.setTargetPosition(pos);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(elbow.getCurrentPosition()>pos) {
            elbow.setPower(power);
        }
        else {
            elbow.setPower(-power);
        }

    }

    public void cancel()
    {
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbow2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbow.setPower(0);
        elbow2.setPower(0);
        moving = false;
    }

    public double getElbow1Position() {
        elbowPosition = elbow.getCurrentPosition();
        return elbowPosition;
    }

    public double getElbow2Position() {
        elbow2Position = elbow2.getCurrentPosition();
        return elbow2Position;
    }

    public boolean isBusy(){
        if(moving) {
            return true;
        }
        return false;
    }

}
