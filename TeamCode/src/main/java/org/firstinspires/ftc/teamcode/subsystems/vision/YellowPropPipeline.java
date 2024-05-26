package org.firstinspires.ftc.teamcode.subsystems.vision;

import static org.opencv.core.Core.ROTATE_90_COUNTERCLOCKWISE;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

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
// This code is for yellow
//This is Vision Testing

@Config
public class YellowPropPipeline implements VisionProcessor, CameraStreamSource {

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
        RIGHT
    }

    // Lower and upper boundaries for colors, Gray, Cyan and Magenta
    public static Scalar
            lower_gray_bounds  = new Scalar(140, 140, 140, 255),
            upper_gray_bounds  = new Scalar(170, 170, 167, 255),
            lower_cyan_bounds    = new Scalar(0, 80, 120, 255),
            upper_cyan_bounds    = new Scalar(60, 255, 255, 255),
            lower_magenta_bounds = new Scalar(155, 20, 60, 255),
            upper_magenta_bounds = new Scalar(255, 60, 100, 255);

    // Color definitions
    private final Scalar
            GRAY = new Scalar(155, 155, 155),
            CYAN    = new Scalar(0, 255, 255),
//            CYAN    = new Scalar(0, 145, 255),
            MAGENTA = new Scalar(255, 0, 255);

    private final Mat grayMat = new Mat();
    private final Mat cyaMat = new Mat();
    private final Mat magMat = new Mat();
    private Mat blurredMatLeft = new Mat();
    private Mat blurredMatCenter = new Mat();


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

        Core.rotate(input, input, ROTATE_90_COUNTERCLOCKWISE);

        Rect leftArea = new Rect(new Point(110,375), new Point(170,430));
        Rect middleArea = new Rect(new Point(445,380), new Point(480,445));

        // Noise reduction
        Imgproc.blur(input, blurredMatLeft, new Size(5, 5));
        blurredMatLeft = blurredMatLeft.submat(leftArea);

        // Apply Morphology
        Mat kernelLeft = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMatLeft, blurredMatLeft, Imgproc.MORPH_CLOSE, kernelLeft);

        // Gets channels from given source mat
        Core.inRange(blurredMatLeft, lower_gray_bounds, upper_gray_bounds, grayMat);
        Core.inRange(blurredMatLeft, lower_cyan_bounds, upper_cyan_bounds, cyaMat);
        Core.inRange(blurredMatLeft, lower_magenta_bounds, upper_magenta_bounds, magMat);

        // Percent and mat definitions
        double grayPercentLeft = Core.countNonZero(grayMat);
        double cyaPercentLeft = Core.countNonZero(cyaMat);
        double magPercentLeft = Core.countNonZero(magMat);

        Imgproc.blur(input, blurredMatCenter, new Size(5, 5));
        blurredMatCenter = blurredMatCenter.submat(middleArea);

        // Apply Morphology
        Mat kernelMiddle = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMatCenter, blurredMatCenter, Imgproc.MORPH_CLOSE, kernelMiddle);

        // Gets channels from given source mat
        Core.inRange(blurredMatCenter, lower_gray_bounds, upper_gray_bounds, grayMat);
        Core.inRange(blurredMatCenter, lower_cyan_bounds, upper_cyan_bounds, cyaMat);
        Core.inRange(blurredMatCenter, lower_magenta_bounds, upper_magenta_bounds, magMat);

        // Gets color specific values
        double grayPercentCenter = Core.countNonZero(grayMat);
        double cyaPercentCenter = Core.countNonZero(cyaMat);
        double magPercentCenter = Core.countNonZero(magMat);

        // Calculates the highest amount of pixels being covered on each side
        double maxPercentLeft = Math.max((grayPercentLeft), (Math.max(cyaPercentLeft, magPercentLeft)));
        double maxPercentCenter = Math.max((grayPercentCenter), (Math.max(cyaPercentCenter, magPercentCenter)));

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected

        if (maxPercentLeft == grayPercentLeft) {
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

       if (maxPercentCenter == grayPercentCenter) {
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

        if ((colorLeft == YellowPropPipeline.ColorDetected.CYAN) || (colorLeft == ColorDetected.MAGENTA)) {
            location = Location.LEFT;
        } else if ((colorMiddle == YellowPropPipeline.ColorDetected.CYAN) || (colorMiddle == YellowPropPipeline.ColorDetected.MAGENTA))
            location = Location.CENTER;
        else
            location = Location.RIGHT;

        return input;
    }

    public Location getLocation() {return location;}

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    protected void finalize() throws Throwable {
        close();
        super.finalize();
    }

    public void close() {

        // Memory cleanup
        blurredMatLeft.release();
        blurredMatCenter.release();
        grayMat.release();
        cyaMat.release();
        magMat.release();

    }

}

