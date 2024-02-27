package org.firstinspires.ftc.teamcode.subsystems.vision.old;

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

@Config
public class SlotPipeline implements VisionProcessor, CameraStreamSource {

    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    public enum ColorDetected {
        YELLOW,
        BLACK
    }

    public enum Location {
        LEFT,
        RIGHT;
    }

    // Lower and upper boundaries for colors
    public static Scalar
            lower_black_bounds  = new Scalar(0, 0, 0, 255),
            upper_black_bounds  = new Scalar(50, 50, 50, 255),
            lower_yellow_bounds    = new Scalar(150, 100, 0, 255),
            upper_yellow_bounds    = new Scalar(255, 255, 130, 255);

    // Color definitions
    private final Scalar
            BLACK = new Scalar(0, 0, 0),
            YELLOW = new Scalar(255, 205, 0);

    // Percent and mat definitions
    private double blackPercentLeft, yellowPercentLeft;
    private Mat blackMat = new Mat(), yelMat = new Mat(), blurredMatLeft = new Mat();

    // Running variable storing the parking position
    private volatile ColorDetected colorLeft;
    private volatile Location location;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {

        Core.rotate(input, input, ROTATE_90_COUNTERCLOCKWISE);
        Rect leftArea = new Rect(new Point(110,375), new Point(170,430));

        // Noise reduction
        Imgproc.blur(input, blurredMatLeft, new Size(5, 5));
        blurredMatLeft = blurredMatLeft.submat(leftArea);

        // Apply Morphology
        Mat kernelLeft = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMatLeft, blurredMatLeft, Imgproc.MORPH_CLOSE, kernelLeft);

        // Gets channels from given source mat
        Core.inRange(blurredMatLeft, lower_black_bounds, upper_black_bounds, blackMat);
        Core.inRange(blurredMatLeft, lower_yellow_bounds, upper_yellow_bounds, yelMat);

        blackPercentLeft = Core.countNonZero(blackMat);
        yellowPercentLeft = Core.countNonZero(yelMat);

        // Calculates the highest amount of pixels being covered on each side
        double maxPercentLeft = Math.max((blackPercentLeft), (yellowPercentLeft));

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected

        if (maxPercentLeft == blackPercentLeft) {
            colorLeft = ColorDetected.BLACK;
            Imgproc.rectangle(
                    input,
                    leftArea,
                    BLACK,
                    2
            );
        } else if (maxPercentLeft == yellowPercentLeft) {
            colorLeft = ColorDetected.YELLOW;
            Imgproc.rectangle(
                    input,
                    leftArea,
                    YELLOW,
                    2
            );
        }


        if ((colorLeft == ColorDetected.YELLOW))
            location = Location.LEFT;
        else
            location = Location.RIGHT;

        return input;
    }

    // Returns an enum being the current position where the robot will park
    public ColorDetected getColorLeft() {
        return colorLeft;
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
        blackMat.release();
        yelMat.release();

    }

}

