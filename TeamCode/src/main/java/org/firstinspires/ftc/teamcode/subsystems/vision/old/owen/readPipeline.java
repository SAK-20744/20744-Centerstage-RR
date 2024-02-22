package org.firstinspires.ftc.teamcode.subsystems.vision.old.owen;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class readPipeline extends OpenCvPipeline
{
    Mat output = new Mat();
    Mat temp = new Mat();
    double HValue, SValue, VValue;
    @Override
    public Mat processFrame(Mat input)
    {
        //Imgproc.blur(input,output,new Size(3,30));

        Point topLeft = new Point( (input.cols() / 2-50)-200, (input.rows() / 2-50) );
        Point bottomRight = new Point((topLeft.x + 50)-200, (topLeft.y+50) );

        Imgproc.cvtColor(input, temp, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(temp, output, Imgproc.COLOR_RGB2HSV_FULL);

        Mat a = new Mat( output, new Rect( topLeft, bottomRight));
        Scalar b = Core.mean(a);

        String HVal = String.format("%1.2f", b.val[0]);
        String SVal = String.format("%1.2f",  b.val[1]);
        String VVal = String.format("%1.2f",  b.val[2]);
        String c = HVal + "," + SVal + "," + VVal;

        HValue = Double.parseDouble(HVal);
        SValue = Double.parseDouble(SVal);
        VValue = Double.parseDouble(VVal);

        Imgproc.putText( input, c, new Point(40,40), 0, 1, new Scalar( 255, 0, 0), 2 );

        Imgproc.rectangle(
                input,
                topLeft,
                bottomRight,
                new Scalar(255, 0, 0), 4);

        return input;
    }
    public int getElementPosition() {
        return 0;
    }
    public double getH() {return HValue;}
    public double getS() {return SValue;}
    public double getV() {return VValue;}
}