package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Base64;

public class lift {

    private DcMotor liftmotor;
    private DigitalChannel liftswitch;
    private Base64.Encoder liftEncoder;
    private boolean liftLimit = true;
    private double liftposition = 0;
    private double liftHigh = 2855;
    private double liftLow = 1330;

    private double liftMedium = 2125;
    private double liftIntake = 50;
    private double liftOff = 1000;
    private double liftTarget = 0;
    private boolean moving = false;
    private double liftPower = 0;
    private boolean liftLimitReset = true;
    private intake Intake;
    private boolean intaking = false;
    private long startTime;

    //milliseconds
    private double errorTime = 2000;


    public lift(HardwareMap hardwareMap, intake NewIntake) {
        liftmotor = hardwareMap.get(DcMotor.class, "lift");
        liftswitch = hardwareMap.get(DigitalChannel.class, "lift switch");
        Intake = NewIntake;
//        liftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "lift"));
        liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public boolean updatelift(){
        liftposition = liftmotor.getCurrentPosition();
        liftLimit = liftswitch.getState();

        if (moving){
            if(!liftmotor.isBusy()){
                moving = false;
/*
                if (intaking){
                    moving = true;
                    liftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftPower = -0.2;
                    liftmotor.setPower(liftPower);
                }*/
            }
        }



        if(liftLimit){
            if (liftLimitReset){
                liftLimitReset = false;
                liftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        else{
            liftLimitReset = true;
        }
        if (intaking && Intake.intakeState()){
            moving = false;
            intaking = false;
        }
        if (intaking && System.currentTimeMillis()>=startTime+errorTime)
        {
            moving = false;
            intaking = false;
        }
        if (intaking && liftLimit){
            moving = false;
            intaking = false;
        }
        if (liftLimit )
        {
            liftPower = Math.min(Math.max(liftPower, 0), 1);
        }
        liftmotor.setPower(liftPower);

        return liftPower != 0;
    }
    public boolean move(double liftSetPower){
        if (liftSetPower!=0){
            moving =false;
        }
        if(!moving) {
            liftPower = -liftSetPower;
            liftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        return liftPower != 0;
    }
    public boolean liftHigh(){
        moving = true;
        liftTarget = liftHigh;
        liftmotor.setTargetPosition((int) liftTarget);
        liftposition = liftmotor.getCurrentPosition();
        liftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (liftTarget < liftposition){

            liftPower=-1;
        }
        else{
            liftPower=1;
        }
        liftmotor.setPower(liftPower);
        return liftPower != 0;
    }

    public boolean liftNum(int height){
        moving = true;
        liftTarget = height;
        liftmotor.setTargetPosition((int) liftTarget);
        liftposition = liftmotor.getCurrentPosition();
        liftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (liftTarget < liftposition){

            liftPower=-1;
        }
        else{
            liftPower=1;
        }
        liftmotor.setPower(liftPower);
        return liftPower != 0;
    }

    public boolean liftMedium(){
        moving = true;
        liftTarget = liftMedium;
        liftmotor.setTargetPosition((int) liftTarget);
        liftposition = liftmotor.getCurrentPosition();
        liftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (liftTarget < liftposition){

            liftPower=-1;
        }
        else{
            liftPower=1;
        }
        liftmotor.setPower(liftPower);
        return liftPower != 0;
    }

    public boolean liftLow(){
        moving = true;
//        intaking = true;
        liftTarget = liftLow;
        liftmotor.setTargetPosition((int) liftTarget);
        liftposition = liftmotor.getCurrentPosition();
        liftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (liftTarget < liftposition){

            liftPower=-1;
        }
        else{
            liftPower=1;
        }
        liftmotor.setPower(liftPower);
        return liftPower != 0;
    }
//    public boolean liftOff(){
//        moving = true;
////        intaking = true;
//        liftTarget = liftOff;
//        liftmotor.setTargetPosition((int) liftTarget);
////        liftposition = liftEncoder.getCurrentPosition();
//        liftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        if (liftTarget < liftposition){
//
//            liftPower=-1;
//        }
//        else{
//            liftPower=1;
//        }
//        liftmotor.setPower(liftPower);
//        return liftPower != 0;
//    }
//    public boolean liftIntake(){
//        moving = true;
////        intaking = true;
//        liftTarget = liftIntake;
//        liftmotor.setTargetPosition((int) liftTarget);
////        liftposition = liftEncoder.getCurrentPosition();
//        liftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        if (liftTarget < liftposition){
//
//            liftPower=-1;
//        }
//        else{
//            liftPower=1;
//        }
//        liftmotor.setPower(liftPower);
//        return liftPower != 0;
//    }
    public void isIntaking()
    {
        moving = true;
        intaking = true;
        startTime = System.currentTimeMillis();

        liftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftPower = -0.15;
        liftmotor.setPower(liftPower);

    }

    public void cancel()
    {
        moving = false;
        intaking = false;
        liftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftPower = 0;
        liftmotor.setPower(liftPower);
    }

    public boolean isBusy(){
        if(!moving) {
            return true;
        }
        return false;
    }
    public double whereAmI()
    {
        liftposition = liftmotor.getCurrentPosition();
        return liftposition;
    }
    public boolean getLimit()
    {
        liftLimit = liftswitch.getState();
        return liftLimit;
    }

}
