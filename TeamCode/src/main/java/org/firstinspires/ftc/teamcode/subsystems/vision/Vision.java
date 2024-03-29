/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.util.trc.FtcEocvColorBlobProcessor;
import org.firstinspires.ftc.teamcode.subsystems.util.trc.FtcVisionEocvColorBlob;
import org.firstinspires.ftc.teamcode.subsystems.util.trc.RobotParams;
import org.firstinspires.ftc.teamcode.subsystems.util.trc.TrcOpenCvColorBlobPipeline;
import org.firstinspires.ftc.teamcode.subsystems.util.trc.TrcVisionTargetInfo;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.imgproc.Imgproc;

/**
 * This class implements AprilTag/TensorFlow/Eocv Vision for the game season. It creates and initializes all the
 * vision target info as well as providing info for the robot, camera and the field. It also provides methods to get
 * the location of the robot and detected targets.
 */
public class Vision
{
    public enum PixelType
    {
        PurplePixel,
        GreenPixel,
        YellowPixel,
        WhitePixel,
        AnyPixel
    }   //enum PixelType

    // Warning: EOCV converts camera stream to RGBA whereas Desktop OpenCV converts it to BGRA. Therefore, the correct
    // color conversion must be RGBA (or RGB) to whatever color space you want to convert.
    //
    // YCrCb Color Space.
    public static final int colorConversion = Imgproc.COLOR_RGB2YCrCb;
    public static final double[] purplePixelColorThresholds = {60.0, 250.0, 120.0, 150.0, 140.0, 170.0};
    public static final double[] greenPixelColorThresholds = {40.0, 200.0, 60.0, 120.0, 60.0, 120.0};
    public static final double[] yellowPixelColorThresholds = {150.0, 250.0, 110.0, 160.0, 20.0, 100.0};
    public static final double[] whitePixelColorThresholds = {245.0, 250.0, 100.0, 130.0, 120.0, 140.0};
    public static final double[] redBlobColorThresholds = {20.0, 120.0, 180.0, 240.0, 90.0, 120.0};
    public static final double[] blueBlobColorThresholds = {20.0, 240.0, 40.0, 254.0, 160.0, 240.0};
//    // HSV Color Space.
//    private static final int colorConversion = Imgproc.COLOR_RGB2HSV_FULL;
//    private static final double[] purplePixelColorThresholds = {170.0, 200.0, 40.0, 160.0, 100.0, 255.0};
//    private static final double[] greenPixelColorThresholds = {60.0, 120.0, 60.0, 255.0, 60.0, 255.0};
//    private static final double[] yellowPixelColorThresholds = {30.0, 60.0, 120.0, 225.0, 200.0, 255.0};
//    private static final double[] whitePixelColorThresholds = {70.0, 120.0, 0.0, 255.0, 230.0, 255.0};
//    private static final double[] redBlobColorThresholds = {0.0, 10.0, 120.0, 255.0, 100.0, 255.0};
//    private static final double[] blueBlobColorThresholds = {160.0, 200.0, 120.0, 255.0, 100.0, 255.0};
    public static final TrcOpenCvColorBlobPipeline.FilterContourParams pixelFilterContourParams =
        new TrcOpenCvColorBlobPipeline.FilterContourParams()
            .setMinArea(1000.0)
            .setMinPerimeter(120.0)
            .setWidthRange(50.0, 1000.0)
            .setHeightRange(10.0, 1000.0)
            .setSolidityRange(0.0, 100.0)
            .setVerticesRange(0.0, 1000.0)
            .setAspectRatioRange(0.2, 5.0);
    public static final TrcOpenCvColorBlobPipeline.FilterContourParams blobFilterContourParams =
        new TrcOpenCvColorBlobPipeline.FilterContourParams()
            .setMinArea(1000.0)
            .setMinPerimeter(100.0)
            .setWidthRange(20.0, 1000.0)
            .setHeightRange(20.0, 1000.0)
            .setSolidityRange(0.0, 100.0)
            .setVerticesRange(0.0, 1000.0)
            .setAspectRatioRange(0.75, 1.25);

//    private FtcRawEocvColorBlobPipeline rawColorBlobPipeline;
//    public FtcRawEocvVision rawColorBlobVision;
//    public FtcVisionAprilTag aprilTagVision;

    private static Pipeline pipeline;
    private static VisionPortal portal;
    private AprilTagProcessor aprilTag;
    private Vision myVision = new Vision();

    private static int lastTeamPropPos = 0;

//    public static FtcVisionEocvColorBlob purplePixelVision;
//    private FtcEocvColorBlobProcessor purplePixelProcessor;
//    public static FtcVisionEocvColorBlob greenPixelVision;
//    private FtcEocvColorBlobProcessor greenPixelProcessor;
//    public FtcVisionEocvColorBlob yellowPixelVision;
//    private FtcEocvColorBlobProcessor yellowPixelProcessor;
//    public FtcVisionEocvColorBlob whitePixelVision;
//    private FtcEocvColorBlobProcessor whitePixelProcessor;
//    public static FtcVisionEocvColorBlob redBlobVision;
//    private FtcEocvColorBlobProcessor redBlobProcessor;
//    public static FtcVisionEocvColorBlob blueBlobVision;
//    private FtcEocvColorBlobProcessor blueBlobProcessor;
//
//    /**
//     * This method returns the last detected team prop position.
//     *
//     * @return last team prop position, 0 if never detected it.
//     */
//    public static int getLastDetectedTeamPropPosition()
//    {
//        return lastTeamPropPos;
//    }   //getLastDetectedTeamPropPosition
//
//    public static int getDetectedTeamPropPosition(boolean red)
//    {
//        int pos = 0;
//        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> teamPropInfo = null;
//
//        if (red)
//        {
//            if (redBlobVision != null)
//            {
//                teamPropInfo = redBlobVision.getBestDetectedTargetInfo(null, null, 0.0, 6);
//            }
//        }
//        else
//        {
//            if (blueBlobVision != null)
//            {
//                teamPropInfo = blueBlobVision.getBestDetectedTargetInfo(null, null, 0.0, 6);
//            }
//        }
//
//        if (teamPropInfo != null)
//        {
//            double teamPropXPos = teamPropInfo.rect.x + teamPropInfo.rect.width/2.0;
//            double oneHalfScreenWidth = RobotParams.CAM_IMAGE_WIDTH/2.0;
//
//            if (teamPropXPos < oneHalfScreenWidth && teamPropXPos > 0)
//            {
//                pos = 1;
//            }
//            else if (teamPropXPos > oneHalfScreenWidth && teamPropXPos < oneHalfScreenWidth*2)
//            {
//                pos = 2;
//            }
//            else
//            {
//                pos = 3;
//            }
//
//        }
//
//        if (pos != 0)
//        {
//            lastTeamPropPos = pos;
//        }
//
//        return pos;
//    }   //getDetectedTeamPropPosition
//
//    public boolean validatePixel(TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> pixelInfo)
//    {
//        // Pixel is 3-inch wide.
//        return pixelInfo.objWidth > 2.0 && pixelInfo.objWidth < 4.0;
//    }   //validatePixel
//
//    /**
//     * This method is called by the Arrays.sort to sort the target object by increasing distance.
//     *
//     * @param a specifies the first target
//     * @param b specifies the second target.
//     * @return negative value if a has closer distance than b, 0 if a and b have equal distances, positive value
//     *         if a has higher distance than b.
//     */
//    private int compareDistance(
//        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> a,
//        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> b)
//    {
//        return (int)((b.objPose.y - a.objPose.y)*100);
//    }   //compareDistance

}