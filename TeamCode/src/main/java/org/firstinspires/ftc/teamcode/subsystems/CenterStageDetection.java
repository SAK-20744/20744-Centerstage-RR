package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection.Location.CENTER;
import static org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection.Location.LEFT;
import static org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection.Location.RIGHT;

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
        GRAY,
        MAGENTA,
        CYAN
    }

    public enum Location {
        LEFT,
        CENTER,
        RIGHT;
    }

    // Lower and upper boundaries for colors
    private static final Scalar
            lower_gray_bounds  = new Scalar(140, 140, 140, 255),
            upper_gray_bounds  = new Scalar(170, 170, 170, 255),
            lower_cyan_bounds    = new Scalar(0, 45, 120, 255),
            upper_cyan_bounds    = new Scalar(100, 255, 255, 255),
            lower_magenta_bounds = new Scalar(140, 0, 0, 255),
            upper_magenta_bounds = new Scalar(255, 150, 150, 255);

    // Color definitions
    private final Scalar
            GRAY = new Scalar(155, 155, 155),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255);

    // Percent and mat definitions
    private double bluPercentLeft, bluPercentCenter, cyaPercentLeft, cyaPercentCenter, magPercentLeft, magPercentCenter;
    private Mat bluMat = new Mat(), cyaMat = new Mat(), magMat = new Mat(), blurredMatLeft = new Mat(), blurredMatCenter = new Mat();

//    // Anchor point definitions
//    Point sleeve_pointA = new Point(
//            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
//            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
//    Point sleeve_pointB = new Point(
//            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Running variable storing the parking position
    private volatile ColorDetected colorLeft;
    private volatile ColorDetected colorMiddle;

    private volatile Location location = CENTER;



    @Override
    public Mat processFrame(Mat input) {

        Rect leftArea = new Rect(new Point(30,155), new Point(80,145));
        Rect middleArea = new Rect(new Point(180,155), new Point(230,145));

        // Noise reduction
        Imgproc.blur(input, blurredMatLeft, new Size(5, 5));
        blurredMatLeft = blurredMatLeft.submat(leftArea);

        // Apply Morphology
        Mat kernelLeft = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMatLeft, blurredMatLeft, Imgproc.MORPH_CLOSE, kernelLeft);

        // Gets channels from given source mat
        Core.inRange(blurredMatLeft, lower_gray_bounds, upper_gray_bounds, bluMat);
        Core.inRange(blurredMatLeft, lower_cyan_bounds, upper_cyan_bounds, cyaMat);
        Core.inRange(blurredMatLeft, lower_magenta_bounds, upper_magenta_bounds, magMat);

        bluPercentLeft = Core.countNonZero(bluMat);
        cyaPercentLeft = Core.countNonZero(cyaMat);
        magPercentLeft = Core.countNonZero(magMat);

        Imgproc.blur(input, blurredMatCenter, new Size(5, 5));
        blurredMatCenter = blurredMatCenter.submat(middleArea);

        // Apply Morphology
        Mat kernelMiddle = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMatCenter, blurredMatCenter, Imgproc.MORPH_CLOSE, kernelMiddle);

        // Gets channels from given source mat
        Core.inRange(blurredMatCenter, lower_gray_bounds, upper_gray_bounds, bluMat);
        Core.inRange(blurredMatCenter, lower_cyan_bounds, upper_cyan_bounds, cyaMat);
        Core.inRange(blurredMatCenter, lower_magenta_bounds, upper_magenta_bounds, magMat);

        // Gets color specific values
        bluPercentCenter = Core.countNonZero(bluMat);
        cyaPercentCenter = Core.countNonZero(cyaMat);
        magPercentCenter = Core.countNonZero(magMat);

        // Calculates the highest amount of pixels being covered on each side
        double maxPercentLeft = Math.max((bluPercentLeft), (Math.max(cyaPercentLeft, magPercentLeft)));
        double maxPercentCenter = Math.max((bluPercentCenter), (Math.max(cyaPercentCenter, magPercentCenter)));

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected

        if (maxPercentLeft == bluPercentLeft) {
            colorLeft = ColorDetected.GRAY;
            Imgproc.rectangle(
                    input,
                    leftArea,
                    GRAY,
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

       if (maxPercentCenter == bluPercentCenter) {
            colorMiddle = ColorDetected.GRAY;
            Imgproc.rectangle(
                    input,
                    middleArea,
                    GRAY,
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
        bluMat.release();
        cyaMat.release();
        magMat.release();


        if ((colorLeft == CenterStageDetection.ColorDetected.CYAN) || (colorLeft == CenterStageDetection.ColorDetected.MAGENTA))
            location = LEFT;
        else if ((colorMiddle == CenterStageDetection.ColorDetected.CYAN) || (colorMiddle == CenterStageDetection.ColorDetected.MAGENTA))
            location = CENTER;
        else
            location = RIGHT;

        return input;
    }

    // Returns an enum being the current position where the robot will park
    public ColorDetected getColorLeft() {
        return colorLeft;
    }

    public ColorDetected getColorMiddle() {
        return colorMiddle;
    }

    public Location getLocation() {return location;}
}

