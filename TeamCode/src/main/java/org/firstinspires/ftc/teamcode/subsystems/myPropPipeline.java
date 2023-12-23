package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

public class myPropPipeline implements VisionProcessor, CameraStreamSource {

    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

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
            lower_gray_bounds  = new Scalar(95, 100, 100, 255),
            upper_gray_bounds  = new Scalar(150, 140, 173, 255),
            lower_cyan_bounds    = new Scalar(0, 80, 180, 255),
            upper_cyan_bounds    = new Scalar(95, 255, 255, 255),
            lower_magenta_bounds = new Scalar(170, 50, 70, 255),
            upper_magenta_bounds = new Scalar(255, 110, 120, 255);

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

    private volatile Location location = Location.CENTER;


    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {

        Rect leftArea = new Rect(new Point(160,200), new Point(280,100));
        Rect middleArea = new Rect(new Point(215,480), new Point(315,400));

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


        if ((colorLeft == myPropPipeline.ColorDetected.CYAN) || (colorLeft == myPropPipeline.ColorDetected.MAGENTA))
            location = Location.LEFT;
        else if ((colorMiddle == myPropPipeline.ColorDetected.CYAN) || (colorMiddle == myPropPipeline.ColorDetected.MAGENTA))
            location = Location.CENTER;
        else
            location = Location.RIGHT;

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

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

}

