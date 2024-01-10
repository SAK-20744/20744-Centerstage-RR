package org.firstinspires.ftc.teamcode.subsystems.InverseKinematics;

public class IVKMath {

    private double length1 = 16.5;
    private double length2 = 16.5;
    private double angle;

    public IVKMath(double theAngle){
        angle = theAngle;
    }

    public double backdropX(double height) {
        return (height * Math.sin(angle))-10;
    }
    public double backdropY(double height) {
        return (height * Math.cos(angle));
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
        return 247-(q1(height)+q2(height));
    }

}