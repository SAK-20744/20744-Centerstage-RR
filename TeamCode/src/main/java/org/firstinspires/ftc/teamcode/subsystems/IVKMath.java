package org.firstinspires.ftc.teamcode.subsystems;

public class IVKMath {

    double length1 = 16.5;
    double length2 = 16.0625;

    public IVKMath(){

    }

    public double backdropY(double height) {
        return height * Math.sin(30);
    }

    public double backdropX(double height) {
        return height * Math.cos(30);
    }

    public double q2 (double x, double y) {
        return -Math.acos(((x*x)+(y*y)-(length1*length1)-(length2*length2)) / (2*length1*length2));
    }

    public double q2backdrop (double height) {
        double x = backdropX(height);
        double y = backdropY(height);

        return -Math.acos(((x*x)+(y*y)-(length1*length1)-(length2*length2)) / (2*length1*length2));
    }

    public double q1 (double x, double y) {
        double q2var = q2(x, y);
        return Math.atan( (length2*Math.sin(q2var)) / (length2*Math.cos(q2var)+ length1) );
    }

    public double q1Backdrop (double height) {
        double x = backdropX(height);
        double y = backdropY(height);

        double q2var = q2(x, y);
        return Math.atan( (length2*Math.sin(q2var)) / (length2*Math.cos(q2var)+ length1) );
    }

    public double q3 (double q1, double q2) {
        return 150 - (q1+q2);
    }

    public double q3Backdrop (double height) {
        double x = backdropX(height);
        double y = backdropY(height);

        double myQ1 = q1(x, y);
        double myQ2 = q2(x,y);

        double myQ3 = q3(myQ1, myQ1);

        return myQ3;
    }
}