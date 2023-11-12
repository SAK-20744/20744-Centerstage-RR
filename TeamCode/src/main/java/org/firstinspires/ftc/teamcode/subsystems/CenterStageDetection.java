package org.firstinspires.ftc.teamcode.subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CenterStageDetection extends OpenCvPipeline {
/*
YELLOW  = Parking Left
CYAN    = Parking Middle
MAGENTA = Parking Right
 */

    public enum ColorDetected {
        RED,
        BLUE,
        MAGENTA,
        CYAN
    }



//    // TOPLEFT anchor point for the bounding box
//    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(85, 168);

    // Width and height for the bounding box
    public static int REGION_WIDTH = 30;
    public static int REGION_HEIGHT = 50;

    // Lower and upper boundaries for colors
    private static final Scalar
            lower_red_bounds  = new Scalar(170, 0, 0, 255),
            upper_red_bounds  = new Scalar(255, 70, 70, 255),
            lower_blue_bounds  = new Scalar(0, 0, 170, 255),
            upper_blue_bounds  = new Scalar(70, 70, 255, 255),
            lower_cyan_bounds    = new Scalar(0, 100, 100, 255),
            upper_cyan_bounds    = new Scalar(170, 255, 255, 255),
            lower_magenta_bounds = new Scalar(170, 0, 170, 255),
            upper_magenta_bounds = new Scalar(255, 70, 255, 255);

    /*
    lower_yellow_bounds  = new Scalar(200, 200, 0, 255),
            upper_yellow_bounds  = new Scalar(255, 255, 130, 255),
            lower_cyan_bounds    = new Scalar(0, 200, 200, 255),
            upper_cyan_bounds    = new Scalar(150, 255, 255, 255),
            lower_magenta_bounds = new Scalar(170, 0, 170, 255),
            upper_magenta_bounds = new Scalar(255, 60, 255, 255);
     */

    // Color definitions
    private final Scalar
            RED  = new Scalar(255, 0, 0),
            BlUE = new Scalar(0, 0, 255),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255);


    // Percent and mat definitions
    private double redPercentLeft, redPercentCenter, bluPercentLeft, bluPercentCenter, cyaPercentLeft, cyaPercentCenter, magPercentLeft, magPercentCenter;
    private Mat redMat = new Mat(), bluMat = new Mat(), cyaMat = new Mat(), magMat = new Mat(), blurredMatLeft = new Mat(), blurredMatCenter = new Mat();

//    // Anchor point definitions
//    Point sleeve_pointA = new Point(
//            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
//            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
//    Point sleeve_pointB = new Point(
//            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Running variable storing the parking position
    private volatile ColorDetected colorLeft = ColorDetected.RED;
    private volatile ColorDetected colorMiddle = ColorDetected.RED;



    @Override
    public Mat processFrame(Mat input) {

        Rect leftArea = new Rect(new Point(10,100), new Point(105,200));
        Rect middleArea = new Rect(new Point(120,100), new Point(205,200));

        // Noise reduction
        Imgproc.blur(input, blurredMatLeft, new Size(5, 5));
        blurredMatLeft = blurredMatLeft.submat(leftArea);

        // Apply Morphology
        Mat kernelLeft = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMatLeft, blurredMatLeft, Imgproc.MORPH_CLOSE, kernelLeft);

        // Gets channels from given source mat
        Core.inRange(blurredMatLeft, lower_red_bounds, upper_red_bounds, redMat);
        Core.inRange(blurredMatLeft, lower_blue_bounds, upper_blue_bounds, bluMat);
        Core.inRange(blurredMatLeft, lower_cyan_bounds, upper_cyan_bounds, cyaMat);
        Core.inRange(blurredMatLeft, lower_magenta_bounds, upper_magenta_bounds, magMat);

        redPercentLeft = Core.countNonZero(redMat);
        bluPercentLeft = Core.countNonZero(bluMat);
        cyaPercentLeft = Core.countNonZero(cyaMat);
        magPercentLeft = Core.countNonZero(magMat);

        Imgproc.blur(input, blurredMatCenter, new Size(5, 5));
        blurredMatCenter = blurredMatCenter.submat(middleArea);

        // Apply Morphology
        Mat kernelMiddle = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMatCenter, blurredMatCenter, Imgproc.MORPH_CLOSE, kernelMiddle);

        // Gets channels from given source mat
        Core.inRange(blurredMatCenter, lower_red_bounds, upper_red_bounds, redMat);
        Core.inRange(blurredMatCenter, lower_blue_bounds, upper_blue_bounds, bluMat);
        Core.inRange(blurredMatCenter, lower_cyan_bounds, upper_cyan_bounds, cyaMat);
        Core.inRange(blurredMatCenter, lower_magenta_bounds, upper_magenta_bounds, magMat);

        // Gets color specific values
        redPercentCenter = Core.countNonZero(redMat);
        bluPercentCenter = Core.countNonZero(bluMat);
        cyaPercentCenter = Core.countNonZero(cyaMat);
        magPercentCenter = Core.countNonZero(magMat);

        // Calculates the highest amount of pixels being covered on each side
        double maxPercentLeft = Math.max((Math.max(redPercentLeft, bluPercentLeft)), (Math.max(cyaPercentLeft, magPercentLeft)));
        double maxPercentCenter = Math.max((Math.max(redPercentCenter, bluPercentCenter)), (Math.max(cyaPercentCenter, magPercentCenter)));

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected
        if (maxPercentLeft == redPercentLeft) {
            colorLeft = ColorDetected.RED;
            Imgproc.rectangle(
                    input,
                    leftArea,
                    RED,
                    2
            );
            // switched CYAN and YELLOW and percents

        } else if (maxPercentLeft == bluPercentLeft) {
            colorLeft = ColorDetected.BLUE;
            Imgproc.rectangle(
                    input,
                    leftArea,
                    BlUE,
                    2
            );
        } else if (maxPercentLeft == cyaPercentLeft) {
            colorLeft = ColorDetected.CYAN;
            Imgproc.rectangle(
                    input,
                    leftArea,
                    CYAN,
                    2
            );
        } else if (maxPercentLeft == magPercentLeft) {
            colorLeft = ColorDetected.MAGENTA;
            Imgproc.rectangle(
                    input,
                    leftArea,
                    MAGENTA,
                    2
            );
        }


        if (maxPercentCenter == redPercentCenter) {
            colorMiddle = ColorDetected.RED;
            Imgproc.rectangle(
                    input,
                    middleArea,
                    RED,
                    2
            );
            // switched CYAN and YELLOW and percents

        } else if (maxPercentCenter == bluPercentCenter) {
            colorMiddle = ColorDetected.BLUE;
            Imgproc.rectangle(
                    input,
                    middleArea,
                    BlUE,
                    2
            );
        } else if (maxPercentCenter == cyaPercentCenter) {
            colorMiddle = ColorDetected.CYAN;
            Imgproc.rectangle(
                    input,
                    middleArea,
                    CYAN,
                    2
            );
        } else if (maxPercentCenter == magPercentCenter) {
            colorMiddle = ColorDetected.MAGENTA;
            Imgproc.rectangle(
                    input,
                    middleArea,
                    MAGENTA,
                    2
            );
        }

        // Memory cleanup
        blurredMatLeft.release();
        blurredMatCenter.release();
        redMat.release();
        bluMat.release();
        cyaMat.release();
        magMat.release();

        return input;
    }

    // Returns an enum being the current position where the robot will park
    public ColorDetected getColorLeft() {
        return colorLeft;
    }

    public ColorDetected getColorMiddle() {
        return colorMiddle;
    }
}

