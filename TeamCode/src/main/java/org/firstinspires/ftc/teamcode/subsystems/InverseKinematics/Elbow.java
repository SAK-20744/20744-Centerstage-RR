package org.firstinspires.ftc.teamcode.subsystems.InverseKinematics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elbow {

    private DcMotor elbow;

    private double elbowPosition;

    private double elbowPower = 0;

    private double degreesToTicksFactor = -1000/90;

    private boolean moving;

    public Elbow(HardwareMap hardwareMap) {
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public boolean updateArm1(){
        elbowPosition = elbow.getCurrentPosition();

        if (moving){
            if(!elbow.isBusy()){
                moving = false;
            }
        }

        elbow.setPower(elbowPower);
        return elbowPower != 0;
    }

    public double getDegreesToTicksFactor(double degrees){
        return degreesToTicksFactor * degrees;
    }

    public void resetArm1(){
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean move(double Arm1SetPower){
        if (Arm1SetPower == 0){
            moving = false;
        }
        if(!moving) {
            elbowPower = Arm1SetPower;
            elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        return Arm1SetPower != 0;
    }

    public boolean ArmToPos(int pos, double power){
        moving = true;
        elbow.setTargetPosition(pos);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(elbow.getCurrentPosition()>=pos) {
            elbow.setPower(power);
        }
        else {
            elbow.setPower(-power);
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
        elbow.setPower(0);
        moving = false;
    }

    public double getElbowPosition() {
        elbowPosition = elbow.getCurrentPosition();
        return elbowPosition;
    }

    public boolean isBusy(){
        if(moving) {
            return true;
        }
        return false;
    }

}
