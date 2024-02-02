///*
// * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
// *
// * Permission is hereby granted, free of charge, to any person obtaining a copy
// * of this software and associated documentation files (the "Software"), to deal
// * in the Software without restriction, including without limitation the rights
// * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// * copies of the Software, and to permit persons to whom the Software is
// * furnished to do so, subject to the following conditions:
// *
// * The above copyright notice and this permission notice shall be included in all
// * copies or substantial portions of the Software.
// *
// * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// * SOFTWARE.
// */
//
//package org.firstinspires.ftc.teamcode.subsystems.util.trc;
//
///**
// * This class implements a generic sensor calibrator on a given sensor with the specified data type.
// *
// * @param <D> specifies the sensor data type to be calibrated.
// */
//public class TrcSensorCalibrator<D>
//{
//    private final String instanceName;
//    private final TrcSensor<D> sensor;
//    private final int numAxes;
//    private final D dataType;
//    private final double[] zeroOffsets;
//    private final double[] deadbands;
//
//    /**
//     * Constructor: Creates an instance of the object.
//     *
//     * @param instanceName specifies the instance name.
//     * @param sensor specifies the sensor to perform the calibration on.
//     * @param numAxes specifies the number of axes needs to be calibrated.
//     * @param dataType specifies the sensor data type to be calibrated.
//     */
//    public TrcSensorCalibrator(String instanceName, TrcSensor<D> sensor, int numAxes, D dataType)
//    {
//        this.instanceName = instanceName;
//        this.sensor = sensor;
//        this.numAxes = numAxes;
//        this.dataType = dataType;
//        zeroOffsets = new double[numAxes];
//        deadbands = new double[numAxes];
//    }   //TrcSensorCalibrator
//
//    /**
//     * This method returns the instance name.
//     *
//     * @return instance name.
//     */
//    @Override
//    public String toString()
//    {
//        return instanceName;
//    }   //toString
//
//    /**
//     * This method calibrates the sensor by reading a number of sensor data samples, averaging the data to determine
//     * the zero offset. It also determines the min and max values of the data samples to form the deadband.
//     *
//     * @param numCalSamples specifies the number of calibration sample to take.
//     * @param calInterval specifies the interval between each calibration sample in msec.
//     */
//    public void calibrate(int numCalSamples, long calInterval)
//    {
//        double[] minValues = new double[numAxes];
//        double[] maxValues = new double[numAxes];
//        double[] sums = new double[numAxes];
//
//        for (int i = 0; i < numAxes; i++)
//        {
//            minValues[i] = maxValues[i] = ((TrcSensor.SensorData<Double>)sensor.getRawData(i, dataType)).value;
//            sums[i] = 0.0;
//        }
//
//        for (int n = 0; n < numCalSamples; n++)
//        {
//            for (int i = 0; i < numAxes; i++)
//            {
//                double value = ((TrcSensor.SensorData<Double>)sensor.getRawData(i, dataType)).value;
//                sums[i] += value;
//
//                if (value < minValues[i])
//                {
//                    minValues[i] = value;
//                }
//                else if (value > maxValues[i])
//                {
//                    maxValues[i] = value;
//                }
//            }
//            TrcTimer.sleep(calInterval);
//        }
//
//        for (int i = 0; i < numAxes; i++)
//        {
//            zeroOffsets[i] = sums[i]/numCalSamples;
//            deadbands[i] = maxValues[i] - minValues[i];
//        }
//    }   //calibrate
//
//    /**
//     * This method applies the calibrated result to the raw sensor data and returned the calibrated data.
//     *
//     * @param index specifies the axis of the sensor calibration to be applied to the data.
//     * @param data specifies the raw sensor data to be calibrated.
//     * @return calibrated sensor data.
//     */
//    public double getCalibratedData(int index, double data)
//    {
//        return TrcUtil.applyDeadband(data - zeroOffsets[index], deadbands[index]);
//    }   //getCalibratedData
//
//}   //class TrcSensorCalibrator