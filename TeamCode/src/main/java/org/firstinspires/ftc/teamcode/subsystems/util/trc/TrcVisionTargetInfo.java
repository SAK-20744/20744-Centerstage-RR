/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 * Based on sample code by Robert Atkinson.
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

package org.firstinspires.ftc.teamcode.subsystems.util.trc;

import org.opencv.core.Point;
import org.opencv.core.Rect;

import java.util.Locale;

/**
 * This class calculates and stores the info for a vision detected target.
 */
public class TrcVisionTargetInfo<O extends TrcVisionTargetInfo.ObjectInfo>
{
    /**
     * This interface implements a method to get the rectangle of the detected object. This should be implemented by
     * a vision detector class.
     */
    public interface ObjectInfo
    {
        /**
         * This method returns the rect of the detected object.
         *
         * @return rect of the detected object.
         */
        Rect getRect();

        /**
         * This method returns the area of the detected object.
         *
         * @return area of the detected object.
         */
        double getArea();

        /**
         * This method returns the pose of the detected object relative to the camera.
         *
         * @return pose of the detected object relative to camera.
         */
        TrcPose3D getObjectPose();

        /**
         * This method returns the objects real world width.
         *
         * @return object real world width, null if not supported.
         */
        Double getObjectWidth();

        /**
         * This method returns the objects real world depth.
         *
         * @return object real world depth, null if not supported.
         */
        Double getObjectDepth();

    }   //interface ObjectInfo

    public O detectedObj;
    public Rect rect;
    public double area;
    public TrcPose3D objPose;
    public Double objWidth;
    public Double objDepth;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param detectedObj specifies the detected object.
     * @param homographyMapper specifies the homography mapper, can be null if not provided in which case
     *        distanceFromCamera, targetWidth and horizontalAngle will not be determined.
     * @param objHeightOffset specifies the object height offset above the floor, used by homography. Can be zero if
     *        homographyMapper is null.
     * @param cameraHeight specifies the height of the camera above the floor, used by homography. Can be zero if
     *        homographyMapper is null.
     */
    public TrcVisionTargetInfo(
        O detectedObj, TrcHomographyMapper homographyMapper, double objHeightOffset, double cameraHeight)
    {
        this.detectedObj = detectedObj;
        this.rect = detectedObj.getRect();
        this.area = detectedObj.getArea();

        if (homographyMapper == null)
        {
            // Caller did not provide homography mapper, it means the caller is doing pose/width/depth calculation
            // itself.
            objPose = detectedObj.getObjectPose();
            objWidth = detectedObj.getObjectWidth();
            objDepth = detectedObj.getObjectDepth();
        }
        else
        {
            // Call provided homography mapper, we will use it to calculate the detected object pose.
            Point topLeft = homographyMapper.mapPoint(new Point(rect.x, rect.y));
            Point topRight = homographyMapper.mapPoint(new Point(rect.x + rect.width, rect.y));
            Point bottomLeft = homographyMapper.mapPoint(new Point(rect.x, rect.y + rect.height));
            Point bottomRight = homographyMapper.mapPoint(new Point(rect.x + rect.width, rect.y + rect.height));
            double xDistanceFromCamera = (bottomLeft.x + bottomRight.x)/2.0;
            double yDistanceFromCamera = (bottomLeft.y + bottomRight.y)/2.0;
            double horiAngleRadian = Math.atan2(xDistanceFromCamera, yDistanceFromCamera);
            double horizontalAngle = Math.toDegrees(horiAngleRadian);
            if (objHeightOffset > 0.0)
            {
                // If object is elevated off the ground, the object distance would be further than it actually is.
                // Therefore, we need to calculate the distance adjustment to be subtracted from the Homography
                // reported distance. Imagine the camera is the sun casting a shadow on the object to the ground.
                // The shadow length is the distance adjustment.
                //
                //  cameraHeight / homographyDistance = objHeightOffset / adjustment
                //  adjustment = objHeightOffset * homographyDistance / cameraHeight
                double adjustment =
                    objHeightOffset * TrcUtil.magnitude(xDistanceFromCamera, yDistanceFromCamera) / cameraHeight;
                xDistanceFromCamera -= adjustment * Math.sin(horiAngleRadian);
                yDistanceFromCamera -= adjustment * Math.cos(horiAngleRadian);
            }
            // Don't have enough info to determine pitch and roll.
            objPose = new TrcPose3D(xDistanceFromCamera, yDistanceFromCamera, objHeightOffset, horizontalAngle, 0.0,
                                   0.0);
            objWidth = bottomRight.x - bottomLeft.x;
            objDepth = ((topLeft.y + topRight.y) - (bottomLeft.y + bottomRight.y))/2.0;
        }
    }   //TrcVisionTargetInfo

    /**
     * Constructor: Create an instance of the object.
     *
     * @param detectedObj specifies the detected object.
     */
    public TrcVisionTargetInfo(O detectedObj)
    {
        this(detectedObj, null, 0.0, 0.0);
    }   //TrcVisionTargetInfo

    /**
     * This method returns the string form of the target info.
     *
     * @return string form of the target info.
     */
    @Override
    public String toString()
    {
        return String.format(
            Locale.US, "(Obj=%s,rect=%s,area=%f,pose=%s,width=%f,depth=%f)",
            detectedObj, rect, area, objPose, objWidth != null? objWidth: 0.0, objDepth != null? objDepth: 0.0);
    }   //toString

}