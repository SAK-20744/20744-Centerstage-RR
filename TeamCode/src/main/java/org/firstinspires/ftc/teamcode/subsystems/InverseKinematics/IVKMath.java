package org.firstinspires.ftc.teamcode.subsystems.InverseKinematics;

public class IVKMath {

    double length1 = 16.5;
    double length2 = 16.5;

    public IVKMath(){

    }

    public double backdropX(double height) {
        return height * Math.sin(60);
    }
    public double backdropY(double height) {
        return height * Math.cos(60);
    }

    public double a1 (double height) {
        return -1*Math.acos((((height*height)+(length1*length1)-(length2*length2))/(2*length1*height)));
    }

    public double q1 (double height) {
        double x = backdropX(height);
        double y = backdropY(height);
        double A1 = a1(height);

        return ((180/Math.PI) * ((Math.atan(y/x))-A1));
    }

    public double q2 (double height) {
        double a2 = -1*Math.acos((((length1*length1)+(length2*length2)-(height*height)) / (2*length1*length2)));

        return (180/Math.PI)*(-1*a2);
    }

    public double q3 (double height) {
        double x = backdropX(height);
        double y = backdropY(height);

        return 240-(q1(height)+q2(height));
    }

    public double getA1 (double height) {
        return a1(height);
    }

    public double getQ1 (double height) {
        return q1(height);
    }
}