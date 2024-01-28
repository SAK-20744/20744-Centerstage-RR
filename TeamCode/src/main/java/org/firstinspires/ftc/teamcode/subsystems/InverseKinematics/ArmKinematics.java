package org.firstinspires.ftc.teamcode.subsystems.InverseKinematics;

public class ArmKinematics {

    private double length1 = 16.5;
    private double length2 = 16.5;
    private double angle;

    // Other methods...

    public double getXCoordinate(double q1, double q2) {
        double q1Radians = Math.toRadians(q1);
        double q2Radians = Math.toRadians(q2);

        return length1 * Math.cos(q1Radians) + length2 * Math.sin(q1Radians + q2Radians);
    }

    public double getYCoordinate(double q1, double q2) {
        double q1Radians = Math.toRadians(q1);
        double q2Radians = Math.toRadians(q2);

        return length1 * Math.sin(q1Radians) + length2 * Math.cos(q1Radians + q2Radians);
    }

}