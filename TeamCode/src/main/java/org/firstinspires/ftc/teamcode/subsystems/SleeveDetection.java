package org.firstinspires.ftc.teamcode.subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SleeveDetection extends OpenCvPipeline {
    /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */
    // TOPLEFT anchor point for the bounding box
    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT;

    // Width and height for the bounding box
    public static int REGION_WIDTH;
    public static int REGION_HEIGHT;
    public SleeveDetection(double topleftAnchorPointX, double topleftAnchorPointY, int width, int height) {
        SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(topleftAnchorPointX, topleftAnchorPointY);
        REGION_WIDTH = width;
        REGION_HEIGHT = height;
    }

    public enum PropColor {
        RED,
        BLUE
    }

    // Lower and upper boundaries for colors
    private static final Scalar
            lower_red_bounds  = new Scalar(100, 100, 0, 255),
            upper_red_bounds  = new Scalar(255, 255, 200, 255),
            lower_blue_bounds    = new Scalar(0, 100, 170, 255),
            upper_blue_bounds    = new Scalar(50, 200, 255, 255);

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
            RED  = new Scalar(255, 255, 0),
            BLUE    = new Scalar(0, 170, 220);

    // Percent and mat definitions
    private double redPercent, bluePercent;
    private Mat redMat = new Mat(), blueMat = new Mat(), blurredMat = new Mat();

    // Anchor point definitions
    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Running variable storing the parking position
    boolean position = false;

    @Override
    public Mat processFrame(Mat input) {
        // Noise reduction
        Imgproc.blur(input, blurredMat, new Size(5, 5));
        blurredMat = blurredMat.submat(new Rect(sleeve_pointA, sleeve_pointB));

        // Apply Morphology
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);

        // Gets channels from given source mat
        Core.inRange(blurredMat, lower_red_bounds, upper_red_bounds, redMat);
        Core.inRange(blurredMat, lower_blue_bounds, upper_blue_bounds, blueMat);

        // Gets color specific values
        redPercent = Core.countNonZero(redMat);
        bluePercent = Core.countNonZero(blueMat);

        // Calculates the highest amount of pixels being covered on each side
        double maxPercent = Math.max(redPercent, bluePercent);

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected
        if (maxPercent == redPercent) {
            position = true;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    RED,
                    2
            );
            // switched CYAN and YELLOW and percents

        } else if (maxPercent == bluePercent) {
            position = true;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    BLUE,
                    2
            );
        } else {
            position = false;
        }

        // Memory cleanup
        blurredMat.release();
        redMat.release();
        blueMat.release();

        return input;
    }

    public boolean getPosition() {
        return position;
    }
    // Returns an enum being the current position where the robot will park
}

