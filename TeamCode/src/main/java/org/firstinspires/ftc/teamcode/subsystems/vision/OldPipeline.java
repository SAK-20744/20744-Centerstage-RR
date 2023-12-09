package org.firstinspires.ftc.teamcode.subsystems.vision;

//import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import static java.lang.Thread.sleep;


//@Config
public class OldPipeline extends OpenCvPipeline {

    private Mat hsvImage;
    private Mat bgrImage;
    private Mat threshold;
    private Mat thresholdAfterROI;
    private Mat dilation;
    private Mat erosion;
    private Mat kernel;
    private Rect roi;

    private Mat colSum;

    private Mat leftImage;
    private Mat centerImage;
    private Mat rightImage;

    private Mat leftBinary;
    private Mat centerBinary;
    private Mat rightBinary;

    private int elementPos = 0;

    private int[] colSumArray;

    private long imageSum = 0;


    private int screenWidth, screenHeight;

    public String allianceColor = "red";
    public static int
            RH_MIN = OldPipeline.RH_MIN,
            RS_MIN = OldPipeline.RS_MIN,
            RV_MIN = OldPipeline.RV_MIN,
            RH_MAX = OldPipeline.RH_MAX,
            RS_MAX = OldPipeline.RS_MAX,
            RV_MAX = OldPipeline.RV_MAX;
    public static int
            BH_MIN = OldPipeline.BH_MIN,
            BS_MIN = OldPipeline.BS_MIN,
            BV_MIN = OldPipeline.BV_MIN,
            BH_MAX = OldPipeline.BH_MAX,
            BS_MAX = OldPipeline.BS_MAX,
            BV_MAX = OldPipeline.BV_MAX;

    public OldPipeline(int screenWidth, int screenHeight) {
        this.screenWidth = screenWidth;
        this.screenHeight = screenHeight;

        hsvImage = new Mat();
        bgrImage = new Mat();
        threshold = new Mat();
        thresholdAfterROI = new Mat();
        dilation = new Mat();
        erosion = new Mat();

        leftImage = new Mat();
        centerImage = new Mat();
        rightImage = new Mat();
        leftBinary = new Mat();
        centerBinary = new Mat();
        rightBinary = new Mat();

        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));

        colSum = new Mat();
        colSumArray = new int[screenWidth];
        roi = new Rect(screenWidth / 3, screenHeight / 3, (int) (1.0 / 3 * screenWidth - 1), 3 * screenHeight / 12);
    }

    public void setAlliance(String alliance) {
        allianceColor = alliance;
    }

    @Override
    public Mat processFrame(Mat input) {
        //synchronized(this) {

        // For whatever reason, OpenCV requires you to do some weird acrobatics to convert
        // RGBA to HSV, that's what is done here
        Imgproc.cvtColor(input, bgrImage, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(bgrImage, hsvImage, Imgproc.COLOR_BGR2HSV_FULL);

        // Threshold hsv values
        if (allianceColor.toLowerCase() == "red") {
            Core.inRange(hsvImage,
                    new Scalar(RH_MIN, RS_MIN, RV_MIN),
                    new Scalar(RH_MAX, RS_MAX, RV_MAX),
                    threshold);
        } else if (allianceColor.toLowerCase() == "blue") {
            Core.inRange(hsvImage,
                    new Scalar(BH_MIN, BS_MIN, BV_MIN),
                    new Scalar(BH_MAX, BS_MAX, BV_MAX),
                    threshold);
        }
        Imgproc.erode(threshold, erosion, kernel);
        Imgproc.dilate(erosion, dilation, kernel);
        Imgproc.erode(dilation, erosion, kernel);
        Imgproc.dilate(erosion, dilation, kernel);
        thresholdAfterROI = dilation.submat(roi);

        Imgproc.cvtColor(dilation, bgrImage, Imgproc.COLOR_GRAY2BGR);
        Imgproc.cvtColor(bgrImage, input, Imgproc.COLOR_BGR2RGBA);
        //Imgproc.rectangle(input, roi, new Scalar(0, 255, 0), 4);

        Core.reduce(thresholdAfterROI, colSum, 0, Core.REDUCE_SUM, CvType.CV_32S);

        long sum = 0;
        colSum.get(0, 0, colSumArray);
        for (int i = 0; i < screenWidth; ++i) {
            sum += colSumArray[i];
        }

        this.setImageSum(sum);

        int imageWidth = input.cols() / 3;
//
//        leftImage = new Mat(input, new Rect(new Point(30,155), new Point(80,145)));
//        centerImage = new Mat(input, new Rect(new Point(180,155), new Point(230,145)));

        Rect leftArea = new Rect(new Point(30,155), new Point(80,145));
        Rect middleArea = new Rect(new Point(180,155), new Point(230,145));
        Rect right = new Rect(new Point(250,155), new Point(300,145));

        leftImage = new Mat(input, new Rect(0, 0, imageWidth, input.rows()));
        centerImage = new Mat(input, new Rect(imageWidth, 0, imageWidth, input.rows()));
        rightImage = new Mat(input, new Rect(imageWidth * 2, 0, imageWidth, input.rows()));

        Core.extractChannel(leftImage, leftBinary, 0);
        Core.extractChannel(centerImage, centerBinary, 0);
        Core.extractChannel(rightImage, rightBinary, 0);

        int leftCount = Core.countNonZero(leftBinary);
        int centerCount = Core.countNonZero(centerBinary);
        int rightCount = Core.countNonZero(rightBinary);

        if ((leftCount > centerCount) && (leftCount > rightCount)) {
            elementPos = 1;
            return leftImage;
        } else if ((centerCount > leftCount) && (centerCount > rightCount)) {
            elementPos = 2;
            return centerImage;
        } else {
            elementPos = 3;
            return input;
        }

        //} //synchronized


        //return input;

        //return leftImage;
    }


    public void setImageSum(long imageSum) {
        this.imageSum = imageSum;
    }

    public long getImageSum() {
        return imageSum;
    }

    public int getElementPosition() {

        return elementPos;
    }

    public void close() {
        hsvImage.release();
        bgrImage.release();
        threshold.release();
        thresholdAfterROI.release();
        dilation.release();
        erosion.release();
        kernel.release();
        colSum.release();
    }
}
