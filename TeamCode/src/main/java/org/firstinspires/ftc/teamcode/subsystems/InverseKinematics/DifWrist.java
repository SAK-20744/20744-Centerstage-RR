package org.firstinspires.ftc.teamcode.subsystems.InverseKinematics;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

public class DifWrist {
    //Details
    String S1Name = "Wrist1";
    String S2Name = "Wrist2";

    //
    private CRServo S1;
    private CRServo S2;
    private double RawPos1;
    private double RawPos2;
    private double vPos1;
    private double vPos2;
    
    private AnalogInput S1AnalogInput;
    private AnalogInput S2AnalogInput;
    private ArrayList<Double> Tasks;
    private ArrayList<Double> Specs;


    public DifWrist(HardwareMap hardwareMap) {
        S1 = hardwareMap.get(CRServo.class, S1Name);
        S2 = hardwareMap.get(CRServo.class, S2Name);
        S1.setDirection(DcMotorSimple.Direction.FORWARD);
        S2.setDirection(DcMotorSimple.Direction.FORWARD);
        S1AnalogInput = hardwareMap.get(AnalogInput.class, "S1A");
        S2AnalogInput = hardwareMap.get(AnalogInput.class, "S2A");
    }

    public void updateWrist() {
        RawPos1 = S1AnalogInput.getVoltage() / S1AnalogInput.getMaxVoltage() * 360;
        RawPos2 = S2AnalogInput.getVoltage() / S2AnalogInput.getMaxVoltage() * 360;
    }

    public void wristMove(double power) {
        S1.setPower(power);
        S2.setPower(power);
    }

    public void wristTurn(double power) {
        S1.setPower(power);
        S2.setPower(-power);
    }

    public void wristMoveToTarget(double target) {

    }

    public void wristTurnToTarget(double target) {

    }

    public void wristToTarget(double moveTarget, double turnTarget) {

    }
}
