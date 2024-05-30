package org.firstinspires.ftc.teamcode.subsystems.vision;

import static org.opencv.core.Core.ROTATE_90_COUNTERCLOCKWISE;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.subsystems.vision.old.BluePropPipeline;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

public class Pipething implements VisionProcessor, CameraStreamSource {

    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {

    }

    public enum ColorDetected {
        GREEN,
        GRAY,
        PURPLE;
    }

    public enum Location {
        LEFT,
        CENTER,
        RIGHT
    }

    // Lower and upper boundaries for colors
    private static final Scalar
            lower_green_bounds  = new Scalar(40, 90, 20, 255),
            upper_green_bounds  = new Scalar(130, 230, 70, 255),
            lower_gray_bounds  = new Scalar(90, 90, 90, 255),
            upper_gray_bounds  = new Scalar(150, 150, 150, 255),
            lower_purple_bounds  = new Scalar(100, 90, 110, 255),
            upper_purple_bounds  = new Scalar(235, 210, 250, 255);

    // Color definitions
    private final Scalar
            GREEN = new Scalar(0, 255, 0),
            GRAY = new Scalar(155, 155, 155),
            PURPLE = new Scalar(216, 193, 235);

    private Mat greenMat = new Mat();
    private Mat grayMat = new Mat();
    private Mat blurredMatCenter = new Mat();

    private volatile ColorDetected colorMiddle;

    // Running variable storing the parking position

    private final Location location = Location.CENTER;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {

//        Core.rotate(input, input, ROTATE_90_COUNTERCLOCKWISE);

        Rect middleArea = new Rect(new Point(300,100), new Point(500,300));

        Imgproc.blur(input, blurredMatCenter, new Size(5, 5));
        blurredMatCenter = blurredMatCenter.submat(middleArea);

        // Apply Morphology
        Mat kernelMiddle = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMatCenter, blurredMatCenter, Imgproc.MORPH_CLOSE, kernelMiddle);

        // Gets channels from given source mat
        Core.inRange(blurredMatCenter, lower_green_bounds, upper_green_bounds, greenMat);
        Core.inRange(blurredMatCenter, lower_gray_bounds, upper_gray_bounds, grayMat);

        // Gets color specific values
        double greenPercentCenter = Core.countNonZero(greenMat);
        double grayPercentCenter = Core.countNonZero(grayMat);

        double maxPercentCenter = Math.max((grayPercentCenter), (greenPercentCenter));


        if (maxPercentCenter == grayPercentCenter) {
            colorMiddle = ColorDetected.GRAY;
            Imgproc.rectangle(
                    input,
                    middleArea,
                    GRAY,
                    2
            );
        } else if (maxPercentCenter == greenPercentCenter) {
            colorMiddle = ColorDetected.GREEN;
            Imgproc.rectangle(
                    input,
                    middleArea,
                    GREEN,
                    2
            );
        }

        // Calculates the highest amount of pixels being covered on each side

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected



        return input;
    }
    @Override
    public void onDrawFrame(Canvas canvas, int i, int i1, float v, float v1, Object o) {


    }

    protected void finalize() throws Throwable {
        close();
        super.finalize();
    }

    public void close() {

        // Memory cleanup
//        blurredMatLeft.release();
        blurredMatCenter.release();
        grayMat.release();
        greenMat.release();

    }

}