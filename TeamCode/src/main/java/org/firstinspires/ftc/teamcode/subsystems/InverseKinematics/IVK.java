package org.firstinspires.ftc.teamcode.subsystems.InverseKinematics;

public class IVK {

    private double length1 = 16.5;
    private double length2 = 16.5;
    private double angle;

    public IVK(){
        return;
    }

    public double a1 (double height) {
        return -1*Math.acos((((height*height)+(length1*length1)-(length2*length2))/(2*length1*height)));
    }

    public double q1 (double x, double y) {
        double hypot = Math.hypot(x,y);
        double A1 = a1(hypot);

        return ((180/Math.PI) * ((Math.atan(y/x))-A1));
    }

    public double q2 (double x, double y) {
        double hypot = Math.hypot(x,y);
        double a2 = -1*Math.acos((((length1*length1)+(length2*length2)-(hypot*hypot)) / (2*length1*length2)));

        return (180/Math.PI)*(-1*a2);
    }

    public double q3 (double x, double y) {
        return 247-(q1(x,y)+q2(x,y));
    }

}