package org.firstinspires.ftc.teamcode.subsystems;

//import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class turret {
    private DigitalChannel turretSwitch;
    private CRServo turretServo;
//    private Encoder turretEncoder;
    private boolean turretLimit = false;
    private double turretPosition = 0.0;
    private double ticksPerDegree = 130.99;
    private double turretPower = 0;
    private double turretTargetTicks = 0;
    private double turretTickLimit = -45000;
    private boolean moving = false;
    private double turretTarget = 0;
    private double turret0position = 0;
    private double turret45position = -5050;
    private double turret225position = -25500;
    private double turret75position = -7000;
    private double turret90position = -8400;
    private double turret80position = -9200;
    private double turret160position = -19300;
    private double turretAutoLeftPosition = -13425;
    private double turret180position = -19600;
    private double turretAUtoRightPosition = -37000;
    private double turret250position = -30500;
    private double turret270position = -34000;
    private double turret315position = -41700;
    private double turretLowAutoStack = -18000;
    private double d = 0.0525;
    private double i = 0.03;
    private double p = 0.000085;
    private DcMotorEx turretMotor;
//    private PIDController controller;
    private boolean PID = false;


    public turret(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(CRServo.class, "turret");
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turretSwitch = hardwareMap.get(DigitalChannel.class, "turret switch");
//        turretEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));
        turretMotor = hardwareMap.get(DcMotorEx.class, "rightRear");
//        controller = new PIDController(p, i, d);
    }

    public boolean updateTurret(){
        turretPosition = turretMotor.getCurrentPosition();
        turretLimit = turretSwitch.getState();
        if (moving){
            if(((turretTargetTicks - 120) <= turretPosition) && (turretPosition <= (turretTargetTicks + 250))){
                turretPower = 0;
                moving = false;
                PID = false;
            }
        }

//        if (PID)
        {
//            double pid = controller.calculate(turretTargetTicks, turretPosition);
//            turretPower = pid;
        }

        if(turretLimit){
            turretPower = Math.min(Math.max(turretPower, 0), 1);
        }

        if(turretPosition <= turretTickLimit){
            turretPower = Math.min(Math.max(turretPower, -1), 0);
        }

        turretServo.setPower(turretPower);
        return turretPower != 0;
    }
//
//    public boolean turnTurret(double RequestedTurretTarget){
//        moving = true;
//        turretTarget = RequestedTurretTarget;
//        turretPosition = turretEncoder.getCurrentPosition();
//        turretTargetTicks = turretPosition + (ticksPerDegree * turretTarget);
//        if (RequestedTurretTarget > 0){
//
//            turretPower=-.7;
//        }
//        else{
//            turretPower=.7;
//        }
//
//        turretServo.setPower(turretPower);
//        return turretPower != 0;
//    }
//
//    public void turretToEncoder(double targetPosition)
//    {
//        moving = true;
//        turretTargetTicks = targetPosition;
//        turretPosition = turretEncoder.getCurrentPosition();
//
//        if (turretTargetTicks < turretPosition)
//        {
//            turretPower= 0.7;
//        }
//        else
//        {
//            turretPower = -0.7;
//        }
//
//        turretServo.setPower(turretPower);
//
//    }
//
//    public void turnSpeedEncoder(double targetPosition, double speed)
//    {
//        moving = true;
//        turretTargetTicks = targetPosition;
//        turretPosition = turretEncoder.getCurrentPosition();
//
//        if(speed>1)
//            speed=1;
//        if (speed<-1)
//            speed=-1;
//
//        if (turretTargetTicks < turretPosition)
//        {
//            turretPower = speed;
//        }
//        else
//        {
//            turretPower = -1 * speed;
//        }
//
//        turretServo.setPower(turretPower);
//
//    }
//
//    public boolean turn0(){
//        moving = true;
//        turretTargetTicks = turret0position;
//        turretPosition = turretEncoder.getCurrentPosition();
//        if (turretTargetTicks > turretPosition){
//
//            turretPower=-.2;
//        }
//        else{
//            turretPower=.2;
//        }
//        turretServo.setPower(turretPower);
//        return turretPower != 0;
//    }
//
//
//    public boolean turn80(){
//        moving = true;
//        turretTargetTicks = turret80position;
//        turretPosition = turretEncoder.getCurrentPosition();
//        if (turretTargetTicks > turretPosition){
//
//            turretPower=-.2;
//        }
//        else{
//            turretPower=.2;
//        }
//        turretServo.setPower(turretPower);
//        return turretPower != 0;
//    }
//
//    public boolean turn225(){
//        moving = true;
//        turretTargetTicks = turret225position;
//        turretPosition = turretEncoder.getCurrentPosition();
//        if (turretTargetTicks > turretPosition){
//
//            turretPower=-.8;
//        }
//        else{
//            turretPower=.8;
//        }
//        turretServo.setPower(turretPower);
//        return turretPower != 0;
//    }
//    public boolean turn45(){
//        moving = true;
//        turretTargetTicks = turret45position;
//        turretPosition = turretEncoder.getCurrentPosition();
//        if (turretTargetTicks > turretPosition){
//
//            turretPower=-.3;
//        }
//        else{
//            turretPower=.3;
//        }
//        turretServo.setPower(turretPower);
//        return turretPower != 0;
//    }
//    public boolean turn75(){
//        moving = true;
//        turretTargetTicks = turret75position;
//        turretPosition = turretEncoder.getCurrentPosition();
//        if (turretTargetTicks > turretPosition){
//
//            turretPower=-.2;
//        }
//        else{
//            turretPower=.2;
//        }
//        turretServo.setPower(turretPower);
//        return turretPower != 0;
//    }
//    public boolean turndropoff(double dropoffPos){
//        moving = true;
//        turretTargetTicks = dropoffPos;
//        turretPosition = turretEncoder.getCurrentPosition();
//        if (turretTargetTicks > turretPosition){
//
//            turretPower=-.2;
//        }
//        else{
//            turretPower=.2;
//        }
//        turretServo.setPower(turretPower);
//        return turretPower != 0;
//    }
//    public boolean turn90(){
//        moving = true;
//        turretTargetTicks = turret90position;
//        turretPosition = turretEncoder.getCurrentPosition();
//        if (turretTargetTicks > turretPosition){
//
//            turretPower=-.2;
//        }
//        else{
//            turretPower=.2;
//        }
//        turretServo.setPower(turretPower);
//        return turretPower != 0;
//    }
//    public boolean turnAutoLeft(){
//        moving = true;
//        turretTargetTicks = turretAutoLeftPosition;
//        turretPosition = turretEncoder.getCurrentPosition();
//        if (turretTargetTicks > turretPosition){
//
//            turretPower=-.2;
//        }
//        else{
//            turretPower=.2;
//        }
//        turretServo.setPower(turretPower);
//        return turretPower != 0;
//    }
//    public boolean turn160(){
//        moving = true;
//        turretTargetTicks = turret160position;
//        turretPosition = turretEncoder.getCurrentPosition();
//        if (turretTargetTicks > turretPosition){
//
//            turretPower=-.2;
//        }
//        else{
//            turretPower=.2;
//        }
//        turretServo.setPower(turretPower);
//        return turretPower != 0;
//    }
//    public boolean turn180(){
//        moving = true;
//        turretTargetTicks = turret180position;
//        turretPosition = turretEncoder.getCurrentPosition();
//        if (turretTargetTicks > turretPosition){
//
//            turretPower=-.2;
//        }
//        else{
//            turretPower=.2;
//        }
//        turretServo.setPower(turretPower);
//        return turretPower != 0;
//    }
//    public boolean turnAutoRight(){
//        moving = true;
//        turretTargetTicks = turretAUtoRightPosition;
//        turretPosition = turretEncoder.getCurrentPosition();
//        if (turretTargetTicks > turretPosition){
//
//            turretPower=-.2;
//        }
//        else{
//            turretPower=.2;
//        }
//        turretServo.setPower(turretPower);
//        return turretPower != 0;
//    }
//    public boolean turn250(){
//        moving = true;
//        turretTargetTicks = turret250position;
//        turretPosition = turretEncoder.getCurrentPosition();
//        if (turretTargetTicks > turretPosition){
//
//            turretPower=-.2;
//        }
//        else{
//            turretPower=.2;
//        }
//        turretServo.setPower(turretPower);
//        return turretPower != 0;
//    }
//    public boolean turn270(){
//        moving = true;
//        turretTargetTicks = turret270position;
//        turretPosition = turretEncoder.getCurrentPosition();
//        if (turretTargetTicks > turretPosition){
//
//            turretPower=-.25;
//        }
//        else{
//            turretPower=.25;
//        }
//        turretServo.setPower(turretPower);
//        return turretPower != 0;
//    }
//    public boolean turn315(){
//        moving = true;
//        turretTargetTicks = turret315position;
//        turretPosition = turretEncoder.getCurrentPosition();
//        if (turretTargetTicks > turretPosition){
//
//            turretPower=-.2;
//        }
//        else{
//            turretPower=.2;
//        }
//        turretServo.setPower(turretPower);
//        return turretPower != 0;
//    }
//    public boolean turnAutoStack(){
//        moving = true;
//        turretTargetTicks = turretLowAutoStack;
//        turretPosition = turretEncoder.getCurrentPosition();
//        if (turretTargetTicks > turretPosition){
//
//            turretPower=-.25;
//        }
//        else{
//            turretPower=.25;
//        }
//        turretServo.setPower(turretPower);
//        return turretPower != 0;
//    }
    public boolean move(float turretSetPower){
        if(!moving) {
            turretPower = turretSetPower;
        }
        return turretPower != 0;
    }

    public boolean isBusy(){
        if(!moving) {
            return true;
        }
        return false;
    }
    public double getTicksPerDegree()
    {
        return ticksPerDegree;
    }
    public double getMaxPosition()
    {
        return turretTickLimit;
    }
    public double whereAmI()
    {
        turretPosition = turretMotor.getCurrentPosition();
        return turretPosition;
    }
    public double getLocation()
    {
        turretPosition = turretMotor.getCurrentPosition();
        return -turretPosition/ticksPerDegree;
    }
    public boolean getLimit()
    {
        turretLimit = turretSwitch.getState();
        return turretLimit;
    }
    public void reset()
    {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
//    public boolean runToProfile(int target)
//    {
//        moving = true;
//        PID = true;
//        turretTargetTicks = target;
//
//        controller.setPID(p,i,d);
//        turretPosition = turretEncoder.getCurrentPosition();
//        double pid = controller.calculate(target, turretPosition);
//
//        turretPower = pid;
//        if(turretLimit){
//            turretPower = Math.min(Math.max(turretPower, 0), 1);
//        }
//        if(turretPosition <= turretTickLimit){
//            turretPower = Math.min(Math.max(turretPower, -1), 0);
//        }
//        turretServo.setPower(turretPower);
//
//        return turretPower != 0;
//    }

//    public double PIDTurret(double target)
//    {
//        double kP = 0;
//        double kI = 0;
//        double kD = 0;
//
//        com.acmerobotics.roadrunner.control.PIDCoefficients coefficients = new com.acmerobotics.roadrunner.control.PIDCoefficients(kP, kI, kD);
//        PIDFController controller = new PIDFController(coefficients);
//
//        controller.setTargetPosition(target);
//
//        double correction = controller.update(turretEncoder.getCurrentPosition());
//
//        return correction;
//    }

}
