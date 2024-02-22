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
//import java.util.ArrayList;
//import java.util.concurrent.atomic.AtomicBoolean;
//
//import TrcCommonLib.trclib.TrcTaskMgr.TaskType;
//
///**
// * This class implements a platform independent generic motor controller. Typically, this class is extended by a
// * platform dependent motor controller class. Not all motor controllers are created equal. Some have more features
// * than the others. This class attempts to simulate some of the features in software. If the platform dependent motor
// * controller supports those features in hardware it should override the corresponding methods and call the hardware
// * directly. For some features that there is no software emulation, this class will throw an
// * UnsupportedOperationException.
// */
//public abstract class TrcMotor implements TrcMotorController, TrcExclusiveSubsystem, TrcOdometrySensor
//{
//    private static final String moduleName = TrcMotor.class.getSimpleName();
//
//    /**
//     * Some actuators are non-linear. The load may vary depending on the position. For example, raising an arm
//     * against gravity will have the maximum load when the arm is horizontal and zero load when vertical. This
//     * caused problem when applying PID control on this kind of actuator because PID controller is only good at
//     * controlling linear actuators. To make PID controller works for non-linear actuators, we need to add power
//     * compensation that counteracts the non-linear component of the load so that PID only deals with the resulting
//     * linear load. However, a generic PID controller doesn't understand the actuator and has no way to come up
//     * with the compensation. Therefore, it is up to the user of the TrcMotor to provide this interface for
//     * computing the output compensation.
//     */
//    public interface PowerCompensation
//    {
//        /**
//         * This method is called to compute the power compensation to counteract the varying non-linear load.
//         *
//         * @param currPower specifies the current motor power.
//         * @return compensation value of the actuator.
//         */
//        double getCompensation(double currPower);
//
//    }   //interface PowerCompensation
//
//    public enum TriggerMode
//    {
//        OnActive,
//        OnInactive,
//        OnBoth
//    }   //enum TriggerMode
//
//    private enum ControlMode
//    {
//        Power,
//        Velocity,
//        Position,
//        Current
//    }   //enum ControlMode
//
//    /**
//     * This class encapsulates all the parameters required to perform an operation. The operation can be a setPower,
//     * setVelocity, setPosition or setCurrent.
//     */
//    private static class TaskParams
//    {
//        ControlMode currControlMode = ControlMode.Power;
//        ControlMode setToControlMode = ControlMode.Power;
//        // If softwarePidCtrl is not null, the operation is a software PID control using this PID controller.
//        TrcPidController softwarePidCtrl = null;
//        Double softwarePidTolerance = null;
//        PowerCompensation powerComp = null;
//        // motorValue can be power, velocity, position or current depending on controlMode.
//        double motorValue = 0.0;
//        // duration is only applicable for Power, Velocity and Current.
//        double duration = 0.0;
//        TrcEvent notifyEvent = null;
//        // holdTarget is only applicable for Position.
//        boolean holdTarget = false;
//        // powerLimit is only applicable for Position.
//        Double powerLimit = 1.0;
//        // timeout is only applicable for Position.
//        double timeout = 0.0;
//        // prevPosTarget is used for setPidPower.
//        Double prevPosTarget = null;
//        double calPower;
//        boolean calibrating = false;
//        // Stall protection.
//        double stallMinPower = 0.0;
//        double stallTolerance = 0.0;
//        double stallTimeout = 0.0;
//        double resetTimeout = 0.0;
//        boolean stalled = false;
//        double prevPos = 0.0;
//        double prevTime = 0.0;
//    }   //class TaskParams
//
//    //
//    // Global objects.
//    //
//    private static final double DEF_BEEP_LOW_FREQUENCY = 440.0;     //in Hz
//    private static final double DEF_BEEP_HIGH_FREQUENCY = 880.0;    //in Hz
//    private static final double DEF_BEEP_DURATION = 0.2;            //in seconds
//
//    private static final ArrayList<TrcMotor> odometryMotors = new ArrayList<>();
//    private static TrcTaskMgr.TaskObject odometryTaskObj;
//    protected static TrcElapsedTimer motorGetPositionElapsedTimer;
//    protected static TrcElapsedTimer motorSetPowerElapsedTimer;
//    protected static TrcElapsedTimer motorSetVelocityElapsedTimer;
//    protected static TrcElapsedTimer motorSetPositionElapsedTimer;
//    protected static TrcElapsedTimer motorSetCurrentElapsedTimer;
//
//    private final ArrayList<TrcMotor> followingMotorsList = new ArrayList<>();
//    private final TaskParams taskParams = new TaskParams();
//
//    protected final TrcDbgTrace tracer;
//    protected final String instanceName;
//    private final TrcDigitalInput lowerLimitSwitch; // for software simulation
//    private final TrcDigitalInput upperLimitSwitch; // for software simulation
//    private final TrcEncoder encoder;               // for software simulation
//    private final TrcOdometrySensor.Odometry odometry;
//    private final TrcTimer timer;
//    private TrcPerformanceTimer pidCtrlTaskPerformanceTimer = null;
//    private boolean odometryEnabled = false;
//    // Configurations for software simulation of motor controller features.
//    private boolean softwarePidEnabled = false;
//    private Double batteryNominalVoltage = null;
//    private boolean limitSwitchesSwapped = false;
//    private boolean lowerLimitSwitchEnabled = false;
//    private boolean upperLimitSwitchEnabled = false;
//    private Double softLowerLimit = null;   // in scaled units.
//    private Double softUpperLimit = null;   // in scaled units.
//    private double sensorScale = 1.0;
//    private double sensorOffset = 0.0;
//    private double zeroPosition = 0.0;      // in scaled units.
//    private double currMotorPower = 0.0;
//    // Used to remember previous set values so to optimize if value set is the same as previous set values.
//    private Double controllerPower;
//    private Double controllerVelocity;
//    private Double controllerPosition;
//    private Double controllerPowerLimit;
//    private Double controllerCurrent;
//    // Software PID controllers.
//    private TrcPidController velPidCtrl = null;
//    private double velTolerance = 0.0;
//    private PowerCompensation velPowerComp = null;
//    private TrcPidController posPidCtrl = null;
//    private double posTolerance = 0.0;
//    private PowerCompensation posPowerComp = null;
//    private TrcPidController currentPidCtrl = null;
//    private double currentTolerance = 0.0;
//    private PowerCompensation currentPowerComp = null;
//    private Double closeLoopControlTarget = null;
//    // Tracer config.
//    private boolean tracePidInfo = false;
//    private boolean verbosePidInfo = false;
//    private TrcRobotBattery battery = null;
//    // Beep device.
//    private TrcTone beepDevice = null;
//    private double beepLowFrequency = DEF_BEEP_LOW_FREQUENCY;
//    private double beepHighFrequency = DEF_BEEP_HIGH_FREQUENCY;
//    private double beepDuration = DEF_BEEP_DURATION;
//    // Reset position on digital trigger support.
//    private TrcTriggerDigitalInput digitalTrigger;
//    private TriggerMode triggerMode;
//    private TrcEvent.Callback triggerCallback;
//    private AtomicBoolean triggerCallbackContext;
//    private TrcEvent triggerCallbackEvent;
//    // Position presets.
//    private double presetTolerance = 2.0;
//    private double[] posPresets = null;
//
//    /**
//     * Constructor: Create an instance of the object.
//     *
//     * @param instanceName specifies the instance name.
//     * @param lowerLimitSwitch specifies an external lower limit switch, can be null if subclass supports it natively.
//     * @param upperLimitSwitch specifies an external forward limit switch, can be null if subclass supports it natively.
//     * @param encoder specifies an external position sensor for reporting motor position, can be null if subclass
//     *        supports it natively.
//     */
//    public TrcMotor(
//        String instanceName, TrcDigitalInput lowerLimitSwitch, TrcDigitalInput upperLimitSwitch, TrcEncoder encoder)
//    {
//        this.tracer = new TrcDbgTrace();
//        this.instanceName = instanceName;
//        this.lowerLimitSwitch = lowerLimitSwitch;
//        this.upperLimitSwitch = upperLimitSwitch;
//        this.encoder = encoder;
//        odometry = new TrcOdometrySensor.Odometry(this);
//        timer = new TrcTimer(instanceName);
//        TrcTaskMgr.TaskObject pidCtrlTaskObj = TrcTaskMgr.createTask(instanceName + ".pidCtrlTask", this::pidCtrlTask);
//        pidCtrlTaskObj.registerTask(TaskType.OUTPUT_TASK);
//
//        if (odometryTaskObj == null)
//        {
//            // Odometry task is a singleton that manages odometry of all motors. The task reads the odometry of all
//            // motors periodically.
//            // This will be a STANDALONE_TASK so that it won't degrade the host task with long delay waiting for the
//            // hardware. If we create individual task for each motor, moving them to STANDALONE_TASK will create too
//            // many threads.
//            odometryTaskObj = TrcTaskMgr.createTask(moduleName + ".odometryTask", TrcMotor::odometryTask);
//            TrcTaskMgr.TaskObject odometryCleanupTaskObj = TrcTaskMgr.createTask(
//                instanceName + ".odometryCleanupTask", this::odometryCleanupTask);
//            odometryCleanupTaskObj.registerTask(TaskType.STOP_TASK);
//        }
//    }   //TrcMotor
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
//     * This method enables/disables performance monitoring of the PID control task.
//     *
//     * @param enabled specifies true to enable, false to disable.
//     */
//    public void setPerformanceMonitorEnabled(boolean enabled)
//    {
//        if (!enabled)
//        {
//            pidCtrlTaskPerformanceTimer = null;
//        }
//        else if (pidCtrlTaskPerformanceTimer == null)
//        {
//            pidCtrlTaskPerformanceTimer = new TrcPerformanceTimer("pidCtrlTask");
//        }
//    }   //setPerformanceMonitorEnabled
//
//    /**
//     * This method sets the message trace level for the tracer.
//     *
//     * @param msgLevel specifies the message level.
//     * @param tracePidInfo specifies true to enable tracing of PID info, false otherwise.
//     * @param verbosePidInfo specifies true to trace verbose PID info, false otherwise.
//     * @param battery specifies the battery object to get battery info for the message.
//     */
//    public void setTraceLevel(
//        TrcDbgTrace.MsgLevel msgLevel, boolean tracePidInfo, boolean verbosePidInfo, TrcRobotBattery battery)
//    {
//        tracer.setTraceLevel(msgLevel);
//        this.tracePidInfo = tracePidInfo;
//        this.verbosePidInfo = verbosePidInfo;
//        this.battery = battery;
//    }   //setTraceLevel
//
//    /**
//     * This method sets the beep device and the beep tones so that it can play beeps when motor stalled.
//     *
//     * @param beepDevice specifies the beep device object.
//     * @param beepLowFrequency specifies the low frequency beep.
//     * @param beepHighFrequency specifies the high frequency beep.
//     * @param beepDuration specifies the beep duration.
//     */
//    public void setBeep(TrcTone beepDevice, double beepLowFrequency, double beepHighFrequency, double beepDuration)
//    {
//        this.beepDevice = beepDevice;
//        this.beepLowFrequency = beepLowFrequency;
//        this.beepHighFrequency = beepHighFrequency;
//        this.beepDuration = beepDuration;
//    }   //setBeep
//
//    /**
//     * This method sets the beep device so that it can play beeps at default frequencies and duration when motor
//     * stalled.
//     *
//     * @param beepDevice specifies the beep device object.
//     */
//    public void setBeep(TrcTone beepDevice)
//    {
//        setBeep(beepDevice, DEF_BEEP_LOW_FREQUENCY, DEF_BEEP_HIGH_FREQUENCY, DEF_BEEP_DURATION);
//    }   //setBeep
//
//    /**
//     * This method converts position sensor units to scaled units.
//     *
//     * @param pos specifies position in sensor units.
//     * @return position in scaled units.
//     */
//    public double convertPositionToScaledUnits(double pos)
//    {
//        return pos*sensorScale - zeroPosition + sensorOffset;
//    }   //convertPositionToScaledUnits
//
//    /**
//     * This method converts position scaled units to sensor units.
//     *
//     * @param pos specifies position in scaled units.
//     * @return position in sensor units.
//     */
//    public double convertPositionToSensorUnits(double pos)
//    {
//        return (pos - sensorOffset + zeroPosition)/sensorScale;
//    }   //convertPositionToSensorUnits
//
//    //
//    // Implements TrcMotorController interface simulating features that the motor controller does not have support
//    // for. If the motor controller does support any of these features, it should override these methods and provide
//    // direct support in hardware.
//    //
//
//    /**
//     * This method enables/disables voltage compensation so that it will maintain the motor output regardless of
//     * battery voltage.
//     *
//     * @param batteryNominalVoltage specifies the nominal voltage of the battery to enable, null to disable.
//     */
//    @Override
//    public void setVoltageCompensationEnabled(Double batteryNominalVoltage)
//    {
//        this.batteryNominalVoltage = batteryNominalVoltage;
//    }   //setVoltageCompensationEnabled
//
//    /**
//     * This method checks if voltage compensation is enabled.
//     *
//     * @return true if voltage compensation is enabled, false if disabled.
//     */
//    @Override
//    public boolean isVoltageCompensationEnabled()
//    {
//        return batteryNominalVoltage != null;
//    }   //isVoltageCompensationEnabled
//
//    /**
//     * This method adds the given motor to the list that will follow this motor. It should only be called by the
//     * given motor to add it to the follower list of the motor it wants to follow.
//     *
//     * @param motor specifies the motor that will follow this motor.
//     */
//    private void addFollower(TrcMotor motor)
//    {
//        synchronized (followingMotorsList)
//        {
//            if (!followingMotorsList.contains(motor))
//            {
//                followingMotorsList.add(motor);
//            }
//        }
//    }   //addFollower
//
//    /**
//     * This method sets this motor to follow another motor.
//     *
//     * @param motor specifies the motor to follow.
//     * @param inverted specifies true if this motor is inverted from the motor it is following, false otherwise.
//     */
//    @Override
//    public void follow(TrcMotor motor, boolean inverted)
//    {
//        motor.addFollower(this);
//        setMotorInverted(motor.isMotorInverted() ^ inverted);
//    }   //follow
//
//    //
//    // TrcMotor only methods. Subclass should not override these methods unless you know what you are doing.
//    //
//
//    /**
//     * This method sets stall protection. When stall protection is turned ON, it will monitor the motor movement for
//     * stalled condition. A motor is considered stalled if:
//     * - the power applied to the motor is above or equal to stallMinPower.
//     * - the motor has not moved or movement stayed within stallTolerance for at least stallTimeout.
//     * Note: By definition, holding target position doing software PID control is stalling. If you decide to enable
//     *       stall protection while holding target, please make sure to set a stallMinPower much greater the power
//     *       necessary to hold position against gravity, for example.
//     *
//     * @param stallMinPower specifies the minimum motor power to detect stalled condition. If the motor power is
//     *                      below stallMinPower, it won't consider it as a stalled condition even if the motor does
//     *                      not move.
//     * @param stallTolerance specifies the movement tolerance within which is still considered stalled.
//     * @param stallTimeout specifies the time in seconds that the motor must stopped before it is declared stalled.
//     * @param resetTimeout specifies the time in seconds the motor must be set to zero power after it is declared
//     *                     stalled will the stalled condition be reset. If this is set to zero, the stalled condition
//     *                     won't be cleared.
//     */
//    public void setStallProtection(
//        double stallMinPower, double stallTolerance, double stallTimeout, double resetTimeout)
//    {
//        synchronized (taskParams)
//        {
//            taskParams.stallMinPower = Math.abs(stallMinPower);
//            taskParams.stallTolerance = Math.abs(stallTolerance);
//            taskParams.stallTimeout = Math.abs(stallTimeout);
//            taskParams.resetTimeout = Math.abs(resetTimeout);
//        }
//    }   //setStallProtection
//
//    /**
//     * This method enables/disables stall detection.
//     *
//     * @param stallDetectionDelay specifies stall detection start delay in seconds, zero to disable stall detection.
//     * @param stallDetectionTimeout specifies stall timeout in seconds which is the minimum elapsed time for the
//     *        motor to be motionless to be considered stalled.
//     * @param stallErrorRateThreshold specifies the error rate threshold below which it will consider stalling.
//     */
//    public void setStallDetectionEnabled(
//        double stallDetectionDelay, double stallDetectionTimeout, double stallErrorRateThreshold)
//    {
//        if (posPidCtrl != null)
//        {
//            posPidCtrl.setStallDetectionEnabled(stallDetectionDelay, stallDetectionTimeout, stallErrorRateThreshold);
//        }
//    }   //setStallDetectionEnabled
//
//    /**
//     * This method enables/disables stall detection.
//     *
//     * @param enabled specifies true to enable stall detection, false to disable.
//     */
//    public void setStallDetectionEnabled(boolean enabled)
//    {
//        if (posPidCtrl != null)
//        {
//            posPidCtrl.setStallDetectionEnabled(enabled);
//        }
//    }   //setStallDetectionEnabled
//
//    /**
//     * This method swaps the forward and reverse limit switches. By default, the lower limit switch is associated
//     * with the reverse limit switch and the upper limit switch is associated with the forward limit switch. This
//     * method will swap the association. Note: if you need to configure the lower and upper limit switches, you must
//     * configure them after this call.
//     *
//     * @param swapped specifies true to swap the limit switches, false otherwise.
//     */
//    public void setLimitSwitchesSwapped(boolean swapped)
//    {
//        limitSwitchesSwapped = swapped;
//    }   //setLimitSwitchesSwapped
//
//    /**
//     * This method enables the lower limit switch and configures it to the specified type.
//     *
//     * @param normalClose specifies true as the normal close switch type, false as normal open.
//     */
//    public void enableLowerLimitSwitch(boolean normalClose)
//    {
//        if (lowerLimitSwitch != null)
//        {
//            lowerLimitSwitchEnabled = true;
//            lowerLimitSwitch.setInverted(normalClose);
//        }
//        else if (limitSwitchesSwapped)
//        {
//            enableMotorFwdLimitSwitch(normalClose);
//        }
//        else
//        {
//            enableMotorRevLimitSwitch(normalClose);
//        }
//    }   //enableLowerLimitSwitch
//
//    /**
//     * This method enables the upper limit switch and configures it to the specified type.
//     *
//     * @param normalClose specifies true as the normal close switch type, false as normal open.
//     */
//    public void enableUpperLimitSwitch(boolean normalClose)
//    {
//        if (upperLimitSwitch != null)
//        {
//            upperLimitSwitchEnabled = true;
//            upperLimitSwitch.setInverted(normalClose);
//        }
//        else if (limitSwitchesSwapped)
//        {
//            enableMotorRevLimitSwitch(normalClose);
//        }
//        else
//        {
//            enableMotorFwdLimitSwitch(normalClose);
//        }
//    }   //enableUpperLimitSwitch
//
//    /**
//     * This method disables the lower limit switch.
//     */
//    public void disableLowerLimitSwitch()
//    {
//        if (lowerLimitSwitch != null)
//        {
//            lowerLimitSwitchEnabled = false;
//        }
//        else if (limitSwitchesSwapped)
//        {
//            disableMotorFwdLimitSwitch();
//        }
//        else
//        {
//            disableMotorRevLimitSwitch();
//        }
//    }   //disableLowerLimitSwitch
//
//    /**
//     * This method disables the upper limit switch.
//     */
//    public void disableUpperLimitSwitch()
//    {
//        if (upperLimitSwitch != null)
//        {
//            upperLimitSwitchEnabled = false;
//        }
//        else if (limitSwitchesSwapped)
//        {
//            disableMotorRevLimitSwitch();
//        }
//        else
//        {
//            disableMotorFwdLimitSwitch();
//        }
//    }   //disableUpperLimitSwitch
//
//    /**
//     * This method inverts the active state of the lower limit switch, typically reflecting whether the switch is
//     * wired normally open or normally close.
//     *
//     * @param inverted specifies true to invert the limit switch to normal close, false to normal open.
//     */
//    public void setLowerLimitSwitchInverted(boolean inverted)
//    {
//        if (lowerLimitSwitch != null)
//        {
//            lowerLimitSwitch.setInverted(inverted);
//        }
//        else if (limitSwitchesSwapped)
//        {
//            setMotorFwdLimitSwitchInverted(inverted);
//        }
//        else
//        {
//            setMotorRevLimitSwitchInverted(inverted);
//        }
//    }   //setLowerLimitSwitchInverted
//
//    /**
//     * This method inverts the active state of the upper limit switch, typically reflecting whether the switch is
//     * wired normally open or normally close.
//     *
//     * @param inverted specifies true to invert the limit switch to normal close, false to normal open.
//     */
//    public void setUpperLimitSwitchInverted(boolean inverted)
//    {
//        if (upperLimitSwitch != null)
//        {
//            upperLimitSwitch.setInverted(inverted);
//        }
//        else if (limitSwitchesSwapped)
//        {
//            setMotorRevLimitSwitchInverted(inverted);
//        }
//        else
//        {
//            setMotorFwdLimitSwitchInverted(inverted);
//        }
//    }   //setFwdLimitSwitchInverted
//
//    /**
//     * This method checks if the lower limit switch is enabled.
//     *
//     * @return true if enabled, false if disabled.
//     */
//    public boolean isLowerLimitSwitchEnabled()
//    {
//        return lowerLimitSwitch != null? lowerLimitSwitchEnabled:
//               limitSwitchesSwapped? isMotorFwdLimitSwitchEnabled(): isMotorRevLimitSwitchEnabled();
//    }   //isLowerLimitSwitchEnabled
//
//    /**
//     * This method checks if the upper limit switch is enabled.
//     *
//     * @return true if enabled, false if disabled.
//     */
//    public boolean isUpperLimitSwitchEnabled()
//    {
//        return upperLimitSwitch != null? upperLimitSwitchEnabled:
//               limitSwitchesSwapped? isMotorRevLimitSwitchEnabled(): isMotorFwdLimitSwitchEnabled();
//    }   //isUpperLimitSwitchEnabled
//
//    /**
//     * This method returns the state of the lower limit switch.
//     *
//     * @return true if lower limit switch is active, false otherwise.
//     */
//    public boolean isLowerLimitSwitchActive()
//    {
//        return lowerLimitSwitch != null? lowerLimitSwitch.isActive():
//               limitSwitchesSwapped? isMotorFwdLimitSwitchActive(): isMotorRevLimitSwitchActive();
//    }   //isLowerLimitSwitchActive
//
//    /**
//     * This method returns the state of the upper limit switch.
//     *
//     * @return true if upper limit switch is active, false otherwise.
//     */
//    public boolean isUpperLimitSwitchActive()
//    {
//        return upperLimitSwitch != null? upperLimitSwitch.isActive():
//               limitSwitchesSwapped? isMotorRevLimitSwitchActive(): isMotorFwdLimitSwitchActive();
//    }   //isUpperLimitSwitchActive
//
//    /**
//     * This method sets the lower and upper soft position limits.
//     *
//     * @param lowerLimit specifies the position of the lower limit in scaled units, null to disable lower limit.
//     * @param upperLimit specifies the position of the upper limit in scaled units, null to disable upper limit.
//     * @param swapped specifies true to swap the direction (lowerLimit is forward and upperLimit is reverse), false
//     *        otherwise. This is only applicable for motor controller soft limits.
//     */
//    public void setSoftPositionLimits(Double lowerLimit, Double upperLimit, boolean swapped)
//    {
//        try
//        {
//            // Convert the limits back to sensor units.
//            lowerLimit = convertPositionToSensorUnits(lowerLimit);
//            upperLimit = convertPositionToSensorUnits(upperLimit);
//            if (swapped)
//            {
//                setMotorRevSoftPositionLimit(upperLimit);
//                setMotorFwdSoftPositionLimit(lowerLimit);
//            }
//            else
//            {
//                setMotorRevSoftPositionLimit(lowerLimit);
//                setMotorFwdSoftPositionLimit(upperLimit);
//            }
//        }
//        catch (UnsupportedOperationException e)
//        {
//            softLowerLimit = lowerLimit;
//            softUpperLimit = upperLimit;
//        }
//    }   //setSoftPositionLimits
//
//    /**
//     * This method inverts the position sensor direction. This may be rare but there are scenarios where the motor
//     * encoder may be mounted somewhere in the power train that it rotates opposite to the motor rotation. This will
//     * cause the encoder reading to go down when the motor is receiving positive power. This method can correct this
//     * situation.
//     *
//     * @param inverted specifies true to invert position sensor direction, false otherwise.
//     */
//    public void setPositionSensorInverted(boolean inverted)
//    {
//        if (encoder != null)
//        {
//            encoder.setInverted(inverted);
//        }
//        else
//        {
//            setMotorPositionSensorInverted(inverted);
//        }
//    }   //setPositionSensorInverted
//
//    /**
//     * This method returns the state of the position sensor direction.
//     *
//     * @return true if the motor direction is inverted, false otherwise.
//     */
//    public boolean isPositionSensorInverted()
//    {
//        if (encoder != null)
//        {
//            return encoder.isInverted();
//        }
//        else
//        {
//            return isMotorPositionSensorInverted();
//        }
//    }   //isPositionSensorInverted
//
//    /**
//     * This method sets the position sensor scale and offset for translating sensor units to real world units.
//     *
//     * @param scale specifies scale factor to multiply the position sensor reading.
//     * @param offset specifies offset added to the scaled sensor reading.
//     * @param zeroOffset specifies the zero offset for absolute encoder.
//     */
//    public void setPositionSensorScaleAndOffset(double scale, double offset, double zeroOffset)
//    {
//        if (encoder != null)
//        {
//            encoder.setScaleAndOffset(scale, offset, zeroOffset);
//        }
//        else
//        {
//            // Motor controllers do support scale but not offset. It's easier for us to just simulate it here.
//            sensorScale = scale;
//            sensorOffset = offset;
//        }
//    }   //setPositionSensorScaleAndOffset
//
//    /**
//     * This method sets the position sensor scale and offset for translating sensor units to real world units.
//     *
//     * @param scale specifies scale factor to multiply the position sensor reading.
//     * @param offset specifies offset added to the scaled sensor reading.
//     */
//    public void setPositionSensorScaleAndOffset(double scale, double offset)
//    {
//        setPositionSensorScaleAndOffset(scale, offset, 0.0);
//    }   //setPositionSensorScaleAndOffset
//
//    /**
//     * This method returns the normalized raw position of the external encoder if one is provided.
//     *
//     * @return external encoder normalized raw position.
//     */
//    public double getEncoderRawPosition()
//    {
//        return encoder != null? encoder.getRawPosition(): 0.0;
//    }   //getEncoderRawPosition
//
//    /**
//     * This method gets the scaled position from an external position sensor if provided. Otherwise, it gets it from
//     * motor controller if it supports one.
//     *
//     * @param zeroAdjust specifies true to zero adjust the position, false otherwise.
//     */
//    private double getControllerPosition(boolean zeroAdjust)
//    {
//        double currPos;
//
//        if (motorGetPositionElapsedTimer != null) motorGetPositionElapsedTimer.recordStartTime();
//        if (encoder != null)
//        {
//            // This is scaled position (i.e. encoder is responsible for scaling).
//            currPos = encoder.getScaledPosition();
//        }
//        else
//        {
//            // Motor controller position sensor is not scaled, we will do scaling here.
//            currPos = getMotorPosition()*sensorScale;
//            if (zeroAdjust)
//            {
//                currPos = currPos - zeroPosition + sensorOffset;
//            }
//        }
//        if (motorGetPositionElapsedTimer != null) motorGetPositionElapsedTimer.recordEndTime();
//
//        return currPos;
//    }   //getControllerPosition
//
//    /**
//     * This method resets the motor position sensor if provided, typically an encoder. Otherwise, it resets the
//     * one on the motor controller if it supports one. If hardware is false, it simulates the reset by reading
//     * the current position as the zero position.
//     *
//     * @param hardware specifies true for resetting hardware position, false for resetting software position.
//     */
//    public void resetPosition(boolean hardware)
//    {
//        if (encoder != null)
//        {
//            // External encoder doesn't support soft reset.
//            encoder.reset();
//        }
//        else
//        {
//            if (hardware)
//            {
//                resetMotorPosition();
//            }
//            zeroPosition = getControllerPosition(false);
//        }
//    }   //resetPosition
//
//    /**
//     * This method resets the motor position sensor, typically an encoder. This method emulates a reset for a
//     * potentiometer by doing a soft reset.
//     */
//    public void resetPosition()
//    {
//        resetPosition(false);
//    }   //resetPosition
//
//    /**
//     * This method enables/disable software PID as the close-loop control.
//     *
//     * @param enabled specifies true to enable software PID control, false to disable.
//     */
//    public void setSoftwarePidEnabled(boolean enabled)
//    {
//        softwarePidEnabled = enabled;
//    }   //setSoftwarePidEnabled
//
//    /**
//     * This method returns the PID control target. It could be a position, velocity or current depending on the
//     * current active PID controller.
//     *
//     * @return PID control target, zero if no active PID controller.
//     */
//    public double getPidTarget()
//    {
//        double target;
//
//        synchronized (taskParams)
//        {
//            if (softwarePidEnabled)
//            {
//                target = taskParams.softwarePidCtrl != null? taskParams.softwarePidCtrl.getTarget(): 0.0;
//            }
//            else
//            {
//                switch (taskParams.currControlMode)
//                {
//                    case Velocity:
//                        target = controllerVelocity;
//                        break;
//
//                    case Position:
//                        target = controllerPosition;
//                        break;
//
//                    case Current:
//                        target = controllerCurrent;
//                        break;
//
//                    default:
//                        // Not in PID control mode.
//                        target = 0.0;
//                        break;
//                }
//            }
//        }
//
//        return target;
//    }   //getPidTarget
//
//    /**
//     * This method checks which PID controller to use for close-loop control.
//     *
//     * @param controlMode specifies the control mode.
//     * @param useSoftwarePid specifies true to use software PID control, false otherwise.
//     * @return the PID controller to used for close-loop control.
//     */
//    private TrcPidController pidCtrlToUse(ControlMode controlMode, boolean useSoftwarePid)
//    {
//        return !useSoftwarePid? null:
//               controlMode == ControlMode.Velocity? velPidCtrl:
//               controlMode == ControlMode.Position? posPidCtrl:
//               controlMode == ControlMode.Current? currentPidCtrl: null;
//    }   //pidCtrlToUse
//
//    /**
//     * This method checks which PID error tolerance to use for close-loop control.
//     *
//     * @param controlMode specifies the control mode.
//     * @param useSoftwarePid specifies true to use software PID control, false otherwise.
//     * @return the PID error tolerance to used for close-loop control.
//     */
//    private Double pidToleranceToUse(ControlMode controlMode, boolean useSoftwarePid)
//    {
//        return !useSoftwarePid? (Double) 0.0:
//               controlMode == ControlMode.Velocity? (Double) velTolerance:
//               controlMode == ControlMode.Position? (Double) posTolerance:
//               controlMode == ControlMode.Current? (Double) currentTolerance: null;
//    }   //pidToleranceToUse
//
//    /**
//     * This method checks which PID Power Compensation callback to use for close-loop control.
//     *
//     * @param controlMode specifies the control mode.
//     * @param useSoftwarePid specifies true to use software PID control, false otherwise.
//     * @return the Power Compensation callback to used for close-loop control.
//     */
//    private PowerCompensation powerCompToUse(ControlMode controlMode, boolean useSoftwarePid)
//    {
//        return !useSoftwarePid? null:
//               controlMode == ControlMode.Velocity? velPowerComp:
//               controlMode == ControlMode.Position? posPowerComp:
//               controlMode == ControlMode.Current? currentPowerComp: null;
//    }   //powerCompToUse
//
//    /**
//     * This method sets the motor control mode and initializes ActionParams appropriately for the control mode.
//     *
//     * @param controlMode specifies the motor control mode.
//     * @param useSoftwarePid specifies true to use software PID control, false to use motor built-in PID.
//     */
//    private void setMotorControlMode(ControlMode controlMode, boolean useSoftwarePid)
//    {
//        synchronized (taskParams)
//        {
//            if (controlMode != taskParams.currControlMode)
//            {
//                // We are changing control mode, clear all previous remembered values.
//                controllerPower = null;
//                controllerVelocity = null;
//                controllerPosition = null;
//                controllerPowerLimit = null;
//                controllerCurrent = null;
//            }
//            taskParams.currControlMode = controlMode;
//            taskParams.softwarePidCtrl = pidCtrlToUse(controlMode, useSoftwarePid);
//            taskParams.softwarePidTolerance = pidToleranceToUse(controlMode, useSoftwarePid);
//            taskParams.powerComp = powerCompToUse(controlMode, useSoftwarePid);
//        }
//    }   //setMotorControlMode
//
//    /**
//     * This method sets the motor power and will do the same to the followers. This method should be used instead of
//     * setMotorPower because this will set the correct control mode and will do the optimization of not sending same
//     * value to the motor repeatedly and also will take care of the followers.
//     *
//     * @param power specifies the percentage power (range -1.0 to 1.0) to be set.
//     * @param changeControlMode specifies true to change control mode, false otherwise.
//     */
//    private void setControllerMotorPower(double power, boolean changeControlMode)
//    {
//        // Optimization: Only do this if we are not already in power control mode or power is different from
//        // last time.
//        if (controllerPower == null || power != controllerPower)
//        {
//            // In most cases, changeControlMode should be true. In rare cases where we want to set motor power while
//            // running a close loop control mode, then we don't want to disturb the close loop control mode. This is
//            // typically used to stop the motor (set motor power to zero) before enabling a close loop control mode.
//            if (changeControlMode)
//            {
//                setMotorControlMode(ControlMode.Power, false);
//                controllerPower = power;
//            }
//            else
//            {
//                controllerPower = null;
//            }
//
//            currMotorPower = power;
//            if (motorSetPowerElapsedTimer != null) motorSetPowerElapsedTimer.recordStartTime();
//            setMotorPower(currMotorPower);
//            if (motorSetPowerElapsedTimer != null) motorSetPowerElapsedTimer.recordEndTime();
//            // Take care of motor followers.
//            synchronized (followingMotorsList)
//            {
//                for (TrcMotor follower : followingMotorsList)
//                {
//                    if (motorSetPowerElapsedTimer != null) motorSetPowerElapsedTimer.recordStartTime();
//                    follower.setMotorPower(currMotorPower);
//                    if (motorSetPowerElapsedTimer != null) motorSetPowerElapsedTimer.recordEndTime();
//                }
//            }
//        }
//    }   //setControllerMotorPower
//
//    /**
//     * This method sets the motor velocity and will do the same to the followers. This method should be used instead
//     * of setMotorVelocity because this will set the correct control mode and do the optimization of not sending same
//     * value to the motor repeatedly and also will take care of the followers.
//     *
//     * @param velocity specifies the velocity in scaled units/sec.
//     */
//    private void setControllerMotorVelocity(double velocity)
//    {
//        // Optimization: Only do this if we are not already in velocity control mode or velocity is different from
//        // last time.
//        if (controllerVelocity == null || velocity != controllerVelocity)
//        {
//            setMotorControlMode(ControlMode.Velocity, false);
//            controllerVelocity = velocity;
//
//            if (motorSetVelocityElapsedTimer != null) motorSetVelocityElapsedTimer.recordStartTime();
//            setMotorVelocity(velocity/sensorScale, 0.0, 0.0);
//            if (motorSetVelocityElapsedTimer != null) motorSetVelocityElapsedTimer.recordEndTime();
//            // pidCtrlTask will take care of followers.
//        }
//    }   //setControllerMotorVelocity
//
//    /**
//     * This method sets the motor position and will do the same to the followers. This method should be used instead
//     * of setMotorPosition because this will set the correct control mode and will take care of the followers.
//     *
//     * @param position specifies the position in scaled units.
//     * @param powerLimit specifies the maximum power output limits.
//     */
//    private void setControllerMotorPosition(double position, double powerLimit)
//    {
//        // Optimization: Only do this if we are not already in position control mode or position is different from
//        // last time.
//        if (controllerPosition == null || controllerPowerLimit == null ||
//            position != controllerPosition || powerLimit != controllerPowerLimit)
//        {
//            // Position mode doesn't have the same optimization as the other control modes.
//            setMotorControlMode(ControlMode.Position, false);
//            controllerPosition = position;
//            controllerPowerLimit = powerLimit;
//
//            if (motorSetPositionElapsedTimer != null) motorSetPositionElapsedTimer.recordStartTime();
//            setMotorPosition(convertPositionToSensorUnits(position), powerLimit, 0.0, 0.0);
//            if (motorSetPositionElapsedTimer != null) motorSetPositionElapsedTimer.recordEndTime();
//            // pidCtrlTask will take care of followers.
//        }
//    }   //setControllerMotorPosition
//
//    /**
//     * This method sets the motor current and will do the same to the followers. This method should be used instead
//     * of setMotorCurrent because this will set the correct control mode and do the optimization of not sending same
//     * value to the motor repeatedly and also will take care of the followers.
//     *
//     * @param current specifies the current in amperes.
//     */
//    private void setControllerMotorCurrent(double current)
//    {
//        // Optimization: Only do this if we are not already in power control mode or power is different from
//        // last time.
//        if (controllerCurrent == null || current != controllerCurrent)
//        {
//            setMotorControlMode(ControlMode.Current, false);
//            controllerCurrent = current;
//
//            if (motorSetCurrentElapsedTimer != null) motorSetCurrentElapsedTimer.recordStartTime();
//            setMotorCurrent(current);
//            if (motorSetCurrentElapsedTimer != null) motorSetCurrentElapsedTimer.recordEndTime();
//            // pidCtrlTask will take care of followers.
//        }
//    }   //setControllerMotorCurrent
//
//    /**
//     * This method commands the motor to run at the given velocity using software PID control.
//     *
//     * @param velocity specifies the velocity in scaled units/sec.
//     */
//    private void setSoftwarePidVelocity(double velocity)
//    {
//        if (velPidCtrl != null)
//        {
//            velPidCtrl.reset();
//            velPidCtrl.setTarget(velocity);
//            velPidCtrl.startStallDetection();
//            setMotorControlMode(ControlMode.Velocity, true);
//        }
//        else
//        {
//            throw new IllegalStateException("Software velocity PID coefficients have not been set.");
//        }
//    }   //setSoftwarePidVelocity
//
//    /**
//     * This method commands the motor to run to the given position using software PID control.
//     *
//     * @param position specifies the position in scaled units.
//     */
//    private void setSoftwarePidPosition(double position)
//    {
//        if (posPidCtrl != null)
//        {
//            posPidCtrl.reset();
//            posPidCtrl.setTarget(position);
//            posPidCtrl.startStallDetection();
//            setMotorControlMode(ControlMode.Position, true);
//        }
//        else
//        {
//            throw new IllegalStateException("Software position PID coefficients have not been set.");
//        }
//    }   //setSoftwarePidPosition
//
//    /**
//     * This method commands the motor to run at the given current using software PID control.
//     *
//     * @param current specifies the current in amperes.
//     */
//    private void setSoftwarePidCurrent(double current)
//    {
//        if (currentPidCtrl != null)
//        {
//            currentPidCtrl.reset();
//            currentPidCtrl.setTarget(current);
//            currentPidCtrl.startStallDetection();
//            setMotorControlMode(ControlMode.Current, true);
//        }
//        else
//        {
//            throw new IllegalStateException("Software current PID coefficients have not been set.");
//        }
//    }   //setSoftwarePidCurrent
//
//    /**
//     * This method cancels a previous operation by resetting the state set by the previous operation. Note: cancel
//     * does not stop the motor and therefore it will still hold its position. If you want to stop the motor, call
//     * the stop method instead.
//     *
//     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
//     *        ownership aware.
//     */
//    public void cancel(String owner)
//    {
//        if (validateOwnership(owner))
//        {
//            timer.cancel();
//            synchronized (taskParams)
//            {
//                taskParams.calibrating = false;
//                if (taskParams.notifyEvent != null)
//                {
//                    taskParams.notifyEvent.cancel();
//                    taskParams.notifyEvent = null;
//                }
//            }
//        }
//    }   //cancel
//
//    /**
//     * This method cancels a previous operation by resetting the state set by the previous operation. Note: cancel
//     * does not stop the motor and therefore it will still hold its position. If you want to stop the motor, call
//     * the stop method instead.
//     */
//    public void cancel()
//    {
//        cancel(null);
//    }   //cancel
//
//    /**
//     * This method stops the motor regardless of the control mode and resets it to power control mode.
//     *
//     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
//     *        ownership aware.
//     */
//    public void stop(String owner)
//    {
//        if (validateOwnership(owner))
//        {
//            cancel();
//            // In addition to canceling the previous operation states, stop the physical motor.
//            setControllerMotorPower(0.0, true);
//        }
//    }   //stop
//
//    /**
//     * This method stops the motor regardless of the control mode and resets it to power control mode.
//     */
//    public void stop()
//    {
//        stop(null);
//    }   //stop
//
//    /**
//     * This method is called when set motor value delay timer has expired. It will set the specified motor value.
//     *
//     * @param context specifies the timer object (not used).
//     */
//    private void delayValueExpiredCallback(Object context)
//    {
//        // Delay timer has expired, set the motor value now.
//        // Note: if delay timer is canceled, there is no callback.
//        TaskParams params = (TaskParams) context;
//
//        synchronized (params)
//        {
//            switch (params.setToControlMode)
//            {
//                case Power:
//                    setControllerMotorPower(params.motorValue, true);
//                    break;
//
//                case Velocity:
//                    closeLoopControlTarget = params.motorValue;
//                    if (softwarePidEnabled)
//                    {
//                        setSoftwarePidVelocity(params.motorValue);
//                    }
//                    else
//                    {
//                        setControllerMotorVelocity(params.motorValue);
//                    }
//                    break;
//
//                case Current:
//                    closeLoopControlTarget = params.motorValue;
//                    if (softwarePidEnabled)
//                    {
//                        setSoftwarePidCurrent(params.motorValue);
//                    }
//                    else
//                    {
//                        setControllerMotorCurrent(params.motorValue);
//                    }
//                    break;
//
//                default:
//                    break;
//            }
//
//            if (params.duration > 0.0)
//            {
//                // We have set a duration, set up a timer for it.
//                timer.set(params.duration, this::durationExpiredCallback, params);
//            }
//            else if (params.notifyEvent != null)
//            {
//                params.notifyEvent.signal();
//                params.notifyEvent = null;
//            }
//        }
//    }   //delayValueExpiredCallback
//
//    /**
//     * This method is called when set motor power duration timer has expired. It will turn the motor off.
//     *
//     * @param context specifies the timer object (not used).
//     */
//    private void durationExpiredCallback(Object context)
//    {
//        // The duration timer has expired, turn everything off and clean up.
//        TaskParams params = (TaskParams) context;
//
//        synchronized (params)
//        {
//            params.duration = 0.0;
//            params.motorValue = 0.0;
//            setControllerMotorPower(0.0, true);
//            if (params.notifyEvent != null)
//            {
//                params.notifyEvent.signal();
//                params.notifyEvent = null;
//            }
//        }
//    }   //durationExpiredCallback
//
//    /**
//     * This method performs voltage compensation. If motor controller supports this natively, batteryNominalVoltage
//     * will be zero and therefore a no-op.
//     *
//     * @param power specifies the power to be compensated.
//     * @return compensated value.
//     */
//    private double voltageCompensation(double power)
//    {
//        if (power != 0.0 && batteryNominalVoltage != null && batteryNominalVoltage > 0.0)
//        {
//            // Only do this if motor controller supports getBusVoltage.
//            try
//            {
//                power *= batteryNominalVoltage / getBusVoltage();
//                power = TrcUtil.clipRange(power);
//            }
//            catch (UnsupportedOperationException e)
//            {
//                tracer.traceWarn(instanceName, "Motor controller does not support getBusVoltage.");
//            }
//        }
//
//        return power;
//    }   //voltageCompensation
//
//    /**
//     * This method is called to set task parameters in an atomic operation.
//     *
//     * @param controlMode specifies motor control mode.
//     * @param motorValue specifies motor value (can be power, velocity, position or current depending on controlMode);
//     * @param duration specifies duration to run the motor in seconds.
//     * @param completionEvent specifies the event to signal when completed.
//     * @param holdTarget specifies true to hold target (applicable for Position mode only).
//     * @param powerLimit specifies the max power applied to the motor (applicable for Position mode only).
//     * @param timeout specifies timeout in seconds for the operation (applicable for Position mode only).
//     */
//    private void setTaskParams(
//        ControlMode controlMode, double motorValue, double duration, TrcEvent completionEvent,
//        boolean holdTarget, Double powerLimit, double timeout)
//    {
//        synchronized (taskParams)
//        {
//            taskParams.setToControlMode = controlMode;
//            taskParams.softwarePidCtrl = pidCtrlToUse(controlMode, softwarePidEnabled);
//            taskParams.softwarePidTolerance = pidToleranceToUse(controlMode, softwarePidEnabled);
//            taskParams.powerComp = powerCompToUse(controlMode, softwarePidEnabled);
//            taskParams.motorValue = motorValue;
//            taskParams.duration = duration;
//            taskParams.notifyEvent = completionEvent;
//            taskParams.holdTarget = holdTarget;
//            taskParams.powerLimit = powerLimit;
//            taskParams.timeout = timeout != 0.0? timeout + TrcTimer.getCurrentTime(): 0.0;
//        }
//    }   //setTaskParams
//
//    /**
//     * This method sets the motor value. The value can be power or velocity. If the motor is not in the correct
//     * control mode, it will stop the motor and set it to the appropriate mode for the value.
//     * Optionally, you can specify a delay before running the motor and a duration for which the motor will be
//     * turned off afterwards.
//     *
//     * @param owner specifies the ID string of the caller for taking ownership, can be null if caller does not
//     *        require ownership.
//     * @param controlMode specifies the motor control mode (Power, Velocity, Current).
//     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
//     * @param value specifies the percentage power (range -1.0 to 1.0) or velocity (scaled units/sec).
//     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
//     *        turning off.
//     * @param completionEvent specifies the event to signal when the motor operation is completed.
//     */
//    private void setMotorValue(
//        String owner, ControlMode controlMode, double delay, double value, double duration, TrcEvent completionEvent)
//    {
//        tracer.traceDebug(
//            instanceName, "owner=%s, controlMode=%s, delay=%.3f, value=%f, duration=%.3f, event=%s",
//            owner, controlMode, delay, value, duration, completionEvent);
//        if (completionEvent != null)
//        {
//            completionEvent.clear();
//        }
//
//        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, tracer);
//        if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;
//
//        if (validateOwnership(owner))
//        {
//            // Cancel previous operation if there is one but keep the physical motor running for a smoother transition.
//            cancel();
//            // Perform voltage compensation only on power control.
//            // If motor controller supports voltage compensation, voltageCompensation will not change the value and
//            // therefore a no-op.
//            if (controlMode == ControlMode.Power)
//            {
//                value = voltageCompensation(value);
//            }
//            // Perform hardware limit switch check.
//            // If motor controller supports hardware limit switches, both lowerLimitSwitch and upperLimitSwitch should
//            // be null and therefore a no-op.
//            if (value < 0.0 && lowerLimitSwitch != null && lowerLimitSwitchEnabled && lowerLimitSwitch.isActive() ||
//                value > 0.0 && upperLimitSwitch != null && upperLimitSwitchEnabled && upperLimitSwitch.isActive())
//            {
//                value = 0.0;
//            }
//            // Perform soft limit check.
//            // If motor controller supports soft limits, softLowerLimit and softUpperLimit will be null and therefore
//            // a no-op.
//            double currPos = getPosition();
//            if (value < 0.0 && softLowerLimit != null && currPos <= softLowerLimit ||
//                value > 0.0 && softUpperLimit != null && currPos >= softUpperLimit)
//            {
//                value = 0.0;
//            }
//
//            setTaskParams(controlMode, value, duration, completionEvent, false, null, 0.0);
//
//            if (delay > 0.0)
//            {
//                // The motor may be spinning, let's stop it but don't change the control mode we already set in
//                // setTaskParams above.
//                setControllerMotorPower(0.0, false);
//                timer.set(delay, this::delayValueExpiredCallback, taskParams);
//            }
//            else
//            {
//                delayValueExpiredCallback(taskParams);
//            }
//        }
//    }   //setMotorValue
//
//    /**
//     * This method sets the motor power. If the motor is not in the correct control mode, it will stop the motor
//     * and set it to power control mode. Optionally, you can specify a delay before running the motor and a duration
//     * for which the motor will be turned off afterwards.
//     *
//     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
//     *        ownership aware.
//     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
//     * @param power specifies the percentage power (range -1.0 to 1.0).
//     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
//     *        turning off.
//     * @param event specifies the event to signal when the motor operation is completed
//     */
//    public void setPower(String owner, double delay, double power, double duration, TrcEvent event)
//    {
//        setMotorValue(owner, ControlMode.Power, delay, power, duration, event);
//    }   //setPower
//
//    /**
//     * This method sets the motor power. If the motor is not in the correct control mode, it will stop the motor
//     * and set it to power control mode. Optionally, you can specify a delay before running the motor and a duration
//     * for which the motor will be turned off afterwards.
//     *
//     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
//     * @param power specifies the percentage power (range -1.0 to 1.0).
//     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
//     *        turning off.
//     * @param event specifies the event to signal when the motor operation is completed
//     */
//    public void setPower(double delay, double power, double duration, TrcEvent event)
//    {
//        setMotorValue(null, ControlMode.Power, delay, power, duration, event);
//    }   //setPower
//
//    /**
//     * This method sets the motor power. If the motor is not in the correct control mode, it will stop the motor
//     * and set it to power control mode. Optionally, you can specify a delay before running the motor and a duration
//     * for which the motor will be turned off afterwards.
//     *
//     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
//     * @param power specifies the percentage power (range -1.0 to 1.0).
//     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
//     *        turning off.
//     */
//    public void setPower(double delay, double power, double duration)
//    {
//        setMotorValue(null, ControlMode.Power, delay, power, duration, null);
//    }   //setPower
//
//    /**
//     * This method sets the motor power. If the motor is not in the correct control mode, it will stop the motor
//     * and set it to power control mode.
//     *
//     * @param power specifies the percentage power (range -1.0 to 1.0).
//     */
//    public void setPower(double power)
//    {
//        setMotorValue(null, ControlMode.Power, 0.0, power, 0.0, null);
//    }   //setPower
//
//    /**
//     * This method returns the motor power.
//     *
//     * @return current motor percentage power (range -1.0 to 1.0).
//     */
//    private double getPower(boolean cached)
//    {
//        return cached? currMotorPower: getMotorPower();
//    }   //getPower
//
//    /**
//     * This method returns the motor power.
//     *
//     * @return current motor percentage power (range -1.0 to 1.0).
//     */
//    public double getPower()
//    {
//        return getPower(false);
//    }   //getPower
//
//    /**
//     * This method sets the motor velocity. If the motor is not in the correct control mode, it will stop the motor
//     * and set it to velocity control mode. Optionally, you can specify a delay before running the motor and a duration
//     * for which the motor will be turned off afterwards.
//     *
//     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
//     *        ownership aware.
//     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
//     * @param velocity specifies velocity in scaled units/sec.
//     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
//     *        turning off.
//     * @param event specifies the event to signal when the motor operation is completed
//     */
//    public void setVelocity(String owner, double delay, double velocity, double duration, TrcEvent event)
//    {
//        setMotorValue(owner, ControlMode.Velocity, delay, velocity, duration, event);
//    }   //setVelocity
//
//    /**
//     * This method sets the motor velocity. If the motor is not in the correct control mode, it will stop the motor
//     * and set it to velocity control mode. Optionally, you can specify a delay before running the motor and a duration
//     * for which the motor will be turned off afterwards.
//     *
//     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
//     * @param velocity specifies velocity in scaled units/sec.
//     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
//     *        turning off.
//     * @param event specifies the event to signal when the motor operation is completed
//     */
//    public void setVelocity(double delay, double velocity, double duration, TrcEvent event)
//    {
//        setMotorValue(null, ControlMode.Velocity, delay, velocity, duration, event);
//    }   //setVelocity
//
//    /**
//     * This method sets the motor velocity. If the motor is not in the correct control mode, it will stop the motor
//     * and set it to power control mode. Optionally, you can specify a delay before running the motor and a duration
//     * for which the motor will be turned off afterwards.
//     *
//     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
//     * @param velocity specifies velocity in scaled units/sec.
//     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
//     *        turning off.
//     */
//    public void setVelocity(double delay, double velocity, double duration)
//    {
//        setMotorValue(null, ControlMode.Velocity, delay, velocity, duration, null);
//    }   //setVelocity
//
//    /**
//     * This method sets the motor velocity. If the motor is not in the correct control mode, it will stop the motor
//     * and set it to power control mode.
//     *
//     * @param velocity specifies velocity in scaled units/sec.
//     */
//    public void setVelocity(double velocity)
//    {
//        setMotorValue(null, ControlMode.Velocity, 0.0, velocity, 0.0, null);
//    }   //setVelocity
//
//    /**
//     * This method returns the motor velocity. Velocity could either be obtained by calling the motor hardware if
//     * it supports it or using the odometry task to monitor the position sensor value. However, accessing hardware
//     * may impact performance because it may involve initiating USB/CAN/I2C bus cycles. Therefore, it may be
//     * beneficial to enable the odometry task to calculate the velocity value.
//     *
//     * @return motor velocity in scaled units/sec.
//     */
//    public double getVelocity()
//    {
//        final double currVel;
//
//        if (odometryEnabled)
//        {
//            synchronized (odometry)
//            {
//                // Don't read from motor hardware directly, get it from odometry instead.
//                currVel = odometry.velocity;
//            }
//        }
//        else
//        {
//            currVel = getMotorVelocity()/sensorScale;
//        }
//
//        return currVel;
//    }   //getVelocity
//
//    /**
//     * This method is called when set motor position delay timer has expired. It will set the specified motor position.
//     *
//     * @param context specifies the timer object (not used).
//     */
//    private void delayPositionExpiredCallback(Object context)
//    {
//        // Delay timer has expired, set the motor value now.
//        TaskParams params = (TaskParams) context;
//
//        synchronized (params)
//        {
//            closeLoopControlTarget = params.motorValue;
//            if (softwarePidEnabled)
//            {
//                // Doing software PID control.
//                // powerLimit is already set in taskParams.
//                setSoftwarePidPosition(params.motorValue);
//            }
//            else
//            {
//                setControllerMotorPosition(params.motorValue, params.powerLimit);
//            }
//        }
//    }   //delayPositionExpiredCallback
//
//    /**
//     * This method sets the motor position. If the motor is not in the correct control mode, it will stop the motor
//     * and set it to power control mode.
//     *
//     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
//     *        ownership aware.
//     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
//     * @param position specifies the position in scaled units to be set.
//     * @param holdTarget specifies true to hold position target, false otherwise.
//     * @param powerLimit specifies the maximum power output limits.
//     * @param completionEvent specifies the event to signal when the motor operation is completed.
//     * @param timeout specifies timeout in seconds.
//     */
//    public void setPosition(
//        String owner, double delay, double position, boolean holdTarget, double powerLimit, TrcEvent completionEvent,
//        double timeout)
//    {
//        tracer.traceDebug(
//            instanceName, "owner=%s, delay=%.3f, pos=%f, holdTarget=%s, powerLimit=%f, event=%s, timeout=%.3f",
//            owner, delay, position, holdTarget, powerLimit, completionEvent, timeout);
//        if (completionEvent != null)
//        {
//            completionEvent.clear();
//        }
//
//        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, tracer);
//        if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;
//
//        if (validateOwnership(owner))
//        {
//            boolean stopIt = false;
//            double currPos = getPosition();
//            boolean forward = position > currPos;
//            // Cancel previous operation if there is one but keep the physical motor running for a smoother transition.
//            cancel();
//            // Perform hardware limit switch check. If motor controller supports hardware limit switches, both
//            // revLimitSwitch and fwdLimitSwitch should be null and therefore a no-op.
//            if (lowerLimitSwitchEnabled && !forward && lowerLimitSwitch.isActive() ||
//                upperLimitSwitchEnabled && forward && upperLimitSwitch.isActive())
//            {
//                stopIt = true;
//            }
//            // Perform soft limit check.
//            // If motor controller supports soft limits, softLowerLimit and softUpperLimit will be null and therefore
//            // a no-op.
//            if (!stopIt && (softLowerLimit != null && position <= softLowerLimit ||
//                            softUpperLimit != null && position >= softUpperLimit))
//            {
//                stopIt = true;
//            }
//
//            if (!stopIt)
//            {
//                setTaskParams(ControlMode.Position, position, 0.0, completionEvent, holdTarget, powerLimit, timeout);
//
//                if (delay > 0.0)
//                {
//                    // The motor may be spinning, let's stop it but don't change the control mode we already set in
//                    // setTaskParams above.
//                    setControllerMotorPower(0.0, false);
//                    timer.set(delay, this::delayPositionExpiredCallback, taskParams);
//                }
//                else
//                {
//                    delayPositionExpiredCallback(taskParams);
//                }
//            }
//        }
//    }   //setPosition
//
//    /**
//     * This method sets the motor position. If the motor is not in the correct control mode, it will stop the motor
//     * and set it to power control mode.
//     *
//     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
//     * @param position specifies the position in scaled units to be set.
//     * @param holdTarget specifies true to hold position target, false otherwise.
//     * @param powerLimit specifies the maximum power output limits.
//     * @param completionEvent specifies the event to signal when the motor operation is completed.
//     * @param timeout specifies timeout in seconds.
//     */
//    public void setPosition(
//        double delay, double position, boolean holdTarget, double powerLimit, TrcEvent completionEvent, double timeout)
//    {
//        setPosition(null, delay, position, holdTarget, powerLimit, completionEvent, timeout);
//    }   //setPosition
//
//    /**
//     * This method sets the motor position. If the motor is not in the correct control mode, it will stop the motor
//     * and set it to power control mode.
//     *
//     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
//     * @param position specifies the position in scaled units to be set.
//     * @param holdTarget specifies true to hold position target, false otherwise.
//     * @param powerLimit specifies the maximum power output limits.
//     * @param completionEvent specifies the event to signal when the motor operation is completed.
//     */
//    public void setPosition(
//        double delay, double position, boolean holdTarget, double powerLimit, TrcEvent completionEvent)
//    {
//        setPosition(null, delay, position, holdTarget, powerLimit, completionEvent, 0.0);
//    }   //setPosition
//
//    /**
//     * This method sets the motor position. If the motor is not in the correct control mode, it will stop the motor
//     * and set it to power control mode.
//     *
//     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
//     * @param position specifies the position in scaled units to be set.
//     * @param holdTarget specifies true to hold position target, false otherwise.
//     * @param powerLimit specifies the maximum power output limits.
//     */
//    public void setPosition(double delay, double position, boolean holdTarget, double powerLimit)
//    {
//        setPosition(null, delay, position, holdTarget, powerLimit, null, 0.0);
//    }   //setPosition
//
//    /**
//     * This method sets the motor position. If the motor is not in the correct control mode, it will stop the motor
//     * and set it to power control mode.
//     *
//     * @param position specifies the position in scaled units to be set.
//     * @param holdTarget specifies true to hold position target, false otherwise.
//     * @param powerLimit specifies the maximum power output limits.
//     */
//    public void setPosition(double position, boolean holdTarget, double powerLimit)
//    {
//        setPosition(null, 0.0, position, holdTarget, powerLimit, null, 0.0);
//    }   //setPosition
//
//    /**
//     * This method sets the motor position. If the motor is not in the correct control mode, it will stop the motor
//     * and set it to power control mode.
//     *
//     * @param position specifies the position in scaled units to be set.
//     * @param holdTarget specifies true to hold position target, false otherwise.
//     */
//    public void setPosition(double position, boolean holdTarget)
//    {
//        setPosition(null, 0.0, position, holdTarget, 1.0, null, 0.0);
//    }   //setPosition
//
//    /**
//     * This method sets the motor position. If the motor is not in the correct control mode, it will stop the motor
//     * and set it to power control mode.
//     *
//     * @param position specifies the position in scaled units to be set.
//     */
//    public void setPosition(double position)
//    {
//        setPosition(null, 0.0, position, true, 1.0, null, 0.0);
//    }   //setPosition
//
//    /**
//     * This method returns the motor position by reading the position sensor. As a performance optimization, it gets
//     * the position from cached odometry if odometry is enabled.
//     *
//     * @return current motor position in scaled units.
//     */
//    public double getPosition()
//    {
//        double currPos;
//
//        if (odometryEnabled)
//        {
//            synchronized (odometry)
//            {
//                // Don't read from motor hardware directly, get it from odometry instead.
//                // Odometry already took care of zeroPosition adjustment.
//                currPos = odometry.currPos;
//            }
//        }
//        else
//        {
//            currPos = getControllerPosition(true);
//        }
//
//        return currPos;
//    }   //getPosition
//
//    /**
//     * This method sets the motor power with PID control. This is basically the same as setPosition but with
//     * dynamically changing powerLimit. The motor will be under position PID control and the power specifies the
//     * maximum limit of how fast the motor can go. The actual motor power is controlled by a PID controller with
//     * the target either set to minPos or maxPos depending on the direction of the motor. This is very useful in
//     * scenarios such as an elevator where you want to have the elevator controlled by a joystick but would like PID
//     * control to pay attention to the upper and lower limits and slow down when approaching those limits. The joystick
//     * value will specify the maximum limit of the elevator power. So if the joystick is only pushed half way, the
//     * elevator will only go half power even though it is far away from the target.
//     *
//     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
//     * @param power specifies the upper bound power of the motor.
//     * @param minPos specifies the minimum of the position range.
//     * @param maxPos specifies the maximum of the position range.
//     * @param holdTarget specifies true to hold target when speed is set to 0, false otherwise.
//     */
//    public void setPidPower(String owner, double power, double minPos, double maxPos, boolean holdTarget)
//    {
//        if (validateOwnership(owner))
//        {
//            // If power is negative, set the target to minPos. If power is positive, set the target to maxPos. We
//            // only set a new target if the target has changed. (i.e. either the motor changes direction, starting
//            // or stopping).
//            double currPos = getPosition();
//            // currTarget is undetermined if the motor is stop.
//            Double currTarget = power < 0.0? (Double)minPos: power > 0.0? (Double)maxPos: null;
//            power = Math.abs(power);
//
//            synchronized (taskParams)
//            {
//                // Target position changes when:
//                // - Starting: Change target position according to power sign.
//                // - Stopping: Change target position to undetermined and hold current position if necessary.
//                // - Changing direction.
//                if (currTarget != null ^ taskParams.prevPosTarget != null ||            // Starting or stopping.
//                    currTarget != null && !currTarget.equals(taskParams.prevPosTarget)) // Changing direction.
//                {
//                    if (currTarget == null)
//                    {
//                        // We are stopping, Relax the power range to max range so we have full power to hold target if
//                        // necessary.
//                        if (holdTarget)
//                        {
//                            // Hold target at current position.
//                            setPosition(0.0, currPos, true, 1.0, null, 0.0);
//                            tracer.traceDebug(
//                                instanceName, "Holding: power=%f, currPos=%f, prevTaget=%s",
//                                power, currPos, taskParams.prevPosTarget);
//                        }
//                        else
//                        {
//                            setControllerMotorPower(0.0, true);
//                            tracer.traceDebug(
//                                instanceName, "Stopping: power=%f, currPos=%f, prevTaget=%s",
//                                power, currPos, taskParams.prevPosTarget);
//                        }
//                    }
//                    else
//                    {
//                        // We are starting or changing direction.
//                        setPosition(0.0, currTarget, holdTarget, power, null, 0.0);
//                        tracer.traceDebug(
//                            instanceName, "Start/ChangeDir: power=%f, currPos=%f, target=%s, prevTaget=%s",
//                            power, currPos, currTarget, taskParams.prevPosTarget);
//                    }
//                    taskParams.prevPosTarget = currTarget;
//                }
//                else if (power == 0.0)
//                {
//                    // We remain stopping, keep the power range relaxed in case we are holding previous target.
//                    taskParams.powerLimit = 1.0;
//                }
//                else
//                {
//                    // Direction did not change but we need to update the power range.
//                    taskParams.powerLimit = power;
//                    tracer.traceDebug(
//                        instanceName, "UpdatePower: power=%f, currPos=%f, target=%s, prevTaget=%s",
//                        power, currPos, currTarget, taskParams.prevPosTarget);
//                }
//            }
//        }
//    }   //setPidPower
//
//    /**
//     * This method sets the motor power with PID control. The motor will be under PID control and the power specifies
//     * the upper bound of how fast the motor will spin. The actual motor power is controlled by a PID controller with
//     * the target either set to minPos or maxPos depending on the direction of the motor. This is very useful in
//     * scenarios such as an elevator where you want to have the elevator controlled by a joystick but would like PID
//     * control to pay attention to the upper and lower limits and slow down when approaching those limits. The joystick
//     * value will specify the upper bound of the elevator power. So if the joystick is only pushed half way, the
//     * elevator will only go half power even though it is far away from the target.
//     *
//     * @param power specifies the upper bound power of the motor.
//     * @param minPos specifies the minimum of the position range.
//     * @param maxPos specifies the maximum of the position range.
//     * @param holdTarget specifies true to hold target when speed is set to 0, false otherwise.
//     */
//    public void setPidPower(double power, double minPos, double maxPos, boolean holdTarget)
//    {
//        setPidPower(null, power, minPos, maxPos, holdTarget);
//    }   //setPidPower
//
//    /**
//     * This method sets the motor current. If the motor is not in the correct control mode, it will stop the motor
//     * and set it to current control mode. Optionally, you can specify a delay before running the motor and a duration
//     * for which the motor will be turned off afterwards.
//     *
//     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
//     *        ownership aware.
//     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
//     * @param current specifies the current in amperes.
//     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
//     *        turning off.
//     * @param event specifies the event to signal when the motor operation is completed
//     */
//    public void setCurrent(String owner, double delay, double current, double duration, TrcEvent event)
//    {
//        setMotorValue(owner, ControlMode.Current, delay, current, duration, event);
//    }   //setCurrent
//
//    /**
//     * This method sets the motor current. If the motor is not in the correct control mode, it will stop the motor
//     * and set it to current control mode. Optionally, you can specify a delay before running the motor and a duration
//     * for which the motor will be turned off afterwards.
//     *
//     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
//     * @param current specifies the current in amperes.
//     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
//     *        turning off.
//     * @param event specifies the event to signal when the motor operation is completed
//     */
//    public void setCurrent(double delay, double current, double duration, TrcEvent event)
//    {
//        setMotorValue(null, ControlMode.Current, delay, current, duration, event);
//    }   //setCurrent
//
//    /**
//     * This method sets the motor current. If the motor is not in the correct control mode, it will stop the motor
//     * and set it to current control mode. Optionally, you can specify a delay before running the motor and a duration
//     * for which the motor will be turned off afterwards.
//     *
//     * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
//     * @param current specifies the current in amperes.
//     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
//     *        turning off.
//     */
//    public void setCurrent(double delay, double current, double duration)
//    {
//        setMotorValue(null, ControlMode.Current, delay, current, duration, null);
//    }   //setCurrent
//
//    /**
//     * This method sets the motor current. If the motor is not in the correct control mode, it will stop the motor
//     * and set it to current control mode.
//     *
//     * @param current specifies the current in amperes.
//     */
//    public void setCurrent(double current)
//    {
//        setMotorValue(null, ControlMode.Current, 0.0, current, 0.0, null);
//    }   //setCurrent
//
//    /**
//     * This method returns the motor current.
//     *
//     * @return current motor current in amperes.
//     */
//    public double getCurrent()
//    {
//        return getMotorCurrent();
//    }   //getCurrent
//
//    /**
//     * This method sets the PID coefficients of the motor's velocity PID controller. Note that PID coefficients are
//     * different for software PID and controller built-in PID. If you enable/disable software PID, you need to set
//     * the appropriate PID coefficients accordingly.
//     *
//     * @param pidCoeff specifies the PID coefficients to set.
//     */
//    public void setVelocityPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
//    {
//        if (softwarePidEnabled)
//        {
//            if (velPidCtrl != null)
//            {
//                velPidCtrl.setPidCoefficients(pidCoeff);
//            }
//            else
//            {
//                velPidCtrl = new TrcPidController(instanceName + ".velPidCtrl", pidCoeff, this::getVelocity);
//                // Set to absolute setpoint because velocity PID control is generally absolute.
//                velPidCtrl.setAbsoluteSetPoint(true);
//            }
//        }
//        else
//        {
//            setMotorVelocityPidCoefficients(pidCoeff);
//        }
//    }   //setVelocityPidCoefficients
//
//    /**
//     * This method sets the PID coefficients of the motor's velocity PID controller. Note that PID coefficients are
//     * different for software PID and controller built-in PID. If you enable/disable software PID, you need to set
//     * the appropriate PID coefficients accordingly.
//     *
//     * @param kP specifies the Kp coefficient.
//     * @param kI specifies the Ki coefficient.
//     * @param kD specifies the Kd coefficient.
//     * @param kF specifies the Kf coefficient.
//     * @param iZone specifies IZone, can be 0.0 if not provided.
//     */
//    public void setVelocityPidCoefficients(double kP, double kI, double kD, double kF, double iZone)
//    {
//        setVelocityPidCoefficients(new TrcPidController.PidCoefficients(kP, kI, kD, kF, iZone));
//    }   //setVelocityPidCoefficients
//
//    /**
//     * This method sets the PID coefficients of the motor's velocity PID controller. Note that PID coefficients are
//     * different for software PID and controller built-in PID. If you enable/disable software PID, you need to set
//     * the appropriate PID coefficients accordingly.
//     *
//     * @param kP specifies the Kp coefficient.
//     * @param kI specifies the Ki coefficient.
//     * @param kD specifies the Kd coefficient.
//     * @param kF specifies the Kf coefficient.
//     */
//    public void setVelocityPidCoefficients(double kP, double kI, double kD, double kF)
//    {
//        setVelocityPidCoefficients(new TrcPidController.PidCoefficients(kP, kI, kD, kF, 0.0));
//    }   //setVelocityPidCoefficients
//
//    /**
//     * This method returns the PID coefficients of the motor's velocity PID controller.
//     *
//     * @return PID coefficients of the motor's velocity PID controller.
//     */
//    public TrcPidController.PidCoefficients getVelocityPidCoefficients()
//    {
//        TrcPidController.PidCoefficients pidCoeff;
//
//        if (softwarePidEnabled)
//        {
//            if (velPidCtrl != null)
//            {
//                pidCoeff = velPidCtrl.getPidCoefficients();
//            }
//            else
//            {
//                throw new IllegalStateException("Software Velocity PID coefficients have not been set.");
//            }
//        }
//        else
//        {
//            pidCoeff = getMotorVelocityPidCoefficients();
//        }
//
//        return pidCoeff;
//    }   //getVelocityPidCoefficients
//
//    /**
//     * This method sets the PID tolerance of the motor's velocity PID controller.
//     *
//     * @param tolerance specifies the PID tolerance.
//     */
//    public void setVelocityPidTolerance(double tolerance)
//    {
//        if (softwarePidEnabled)
//        {
//            if (velPidCtrl != null)
//            {
//                velTolerance = tolerance;
//            }
//            else
//            {
//                throw new IllegalStateException("Software Velocity PID coefficients have not been set.");
//            }
//        }
//        else
//        {
//            setMotorVelocityPidTolerance(tolerance);
//            velTolerance = tolerance;
//        }
//    }   //setVelocityPidTolerance
//
//    /**
//     * This method checks if velocity PID control has reached target.
//     *
//     * @param tolerance specifies the PID tolerance.
//     * @return true if velocity has reached target, false otherwise.
//     */
//    public boolean getVelocityOnTarget(double tolerance)
//    {
//        boolean onTarget;
//
//        if (softwarePidEnabled)
//        {
//            if (velPidCtrl != null)
//            {
//                onTarget = velPidCtrl.isOnTarget(tolerance);
//                velTolerance = tolerance;
//            }
//            else
//            {
//                throw new IllegalStateException("Software Velocity PID coefficients have not been set.");
//            }
//        }
//        else
//        {
//            onTarget = getMotorVelocityOnTarget(tolerance);
//            velTolerance = tolerance;
//        }
//
//        return onTarget;
//    }   //getVelocityOnTarget
//
//    /**
//     * This method sets the power compensation callback of the motor's velocity PID controller.
//     *
//     * @param powerComp specifies the power compensation callback.
//     */
//    public void setVelocityPidPowerComp(PowerCompensation powerComp)
//    {
//        if (softwarePidEnabled)
//        {
//            velPowerComp = powerComp;
//        }
//    }   //setVelocityPidPowerComp
//
//    /**
//     * This method returns the software velocity PID controller if one was created by enabling software PID control.
//     * It does not mean software PID control is current enable. It just means software PID control was enabled at
//     * one point. This allows you to configure the software PID controller. For example, whether the target is absolute
//     * or relative, specifying noOscillation etc.
//     *
//     * @return software velocity PID controller.
//     */
//    public TrcPidController getVelocityPidController()
//    {
//        return velPidCtrl;
//    }   //getVelocityPidController
//
//    /**
//     * This method sets the PID coefficients of the motor's position PID controller. Note that PID coefficients are
//     * different for software PID and controller built-in PID. If you enable/disable software PID, you need to set
//     * the appropriate PID coefficients accordingly.
//     *
//     * @param pidCoeff specifies the PID coefficients to set.
//     */
//    public void setPositionPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
//    {
//        if (softwarePidEnabled)
//        {
//            if (posPidCtrl != null)
//            {
//                posPidCtrl.setPidCoefficients(pidCoeff);
//            }
//            else
//            {
//                posPidCtrl = new TrcPidController(instanceName + ".posPidCtrl", pidCoeff, this::getPosition);
//                // Set to absolute setpoint because position PID control is generally absolute.
//                posPidCtrl.setAbsoluteSetPoint(true);
//            }
//        }
//        else
//        {
//            setMotorPositionPidCoefficients(pidCoeff);
//        }
//    }   //setPositionPidCoefficients
//
//    /**
//     * This method sets the PID coefficients of the motor's position PID controller. Note that PID coefficients are
//     * different for software PID and controller built-in PID. If you enable/disable software PID, you need to set
//     * the appropriate PID coefficients accordingly.
//     *
//     * @param kP specifies the Kp coefficient.
//     * @param kI specifies the Ki coefficient.
//     * @param kD specifies the Kd coefficient.
//     * @param kF specifies the Kf coefficient.
//     * @param iZone specifies IZone, can be 0.0 if not provided.
//     */
//    public void setPositionPidCoefficients(double kP, double kI, double kD, double kF, double iZone)
//    {
//        setPositionPidCoefficients(new TrcPidController.PidCoefficients(kP, kI, kD, kF, iZone));
//    }   //setPositionPidCoefficients
//
//    /**
//     * This method sets the PID coefficients of the motor's position PID controller. Note that PID coefficients are
//     * different for software PID and controller built-in PID. If you enable/disable software PID, you need to set
//     * the appropriate PID coefficients accordingly.
//     *
//     * @param kP specifies the Kp coefficient.
//     * @param kI specifies the Ki coefficient.
//     * @param kD specifies the Kd coefficient.
//     * @param kF specifies the Kf coefficient.
//     */
//    public void setPositionPidCoefficients(double kP, double kI, double kD, double kF)
//    {
//        setPositionPidCoefficients(new TrcPidController.PidCoefficients(kP, kI, kD, kF, 0.0));
//    }   //setPositionPidCoefficients
//
//    /**
//     * This method returns the PID coefficients of the motor's position PID controller.
//     *
//     * @return PID coefficients of the motor's position PID controller.
//     */
//    public TrcPidController.PidCoefficients getPositionPidCoefficients()
//    {
//        TrcPidController.PidCoefficients pidCoeff;
//
//        if (softwarePidEnabled)
//        {
//            if (posPidCtrl != null)
//            {
//                pidCoeff = posPidCtrl.getPidCoefficients();
//            }
//            else
//            {
//                throw new IllegalStateException("Software Position PID coefficients have not been set.");
//            }
//        }
//        else
//        {
//            pidCoeff = getMotorPositionPidCoefficients();
//        }
//
//        return pidCoeff;
//    }   //getPositionPidCoefficients
//
//    /**
//     * This method sets the PID tolerance of the motor's position PID controller.
//     *
//     * @param tolerance specifies the PID tolerance.
//     */
//    public void setPositionPidTolerance(double tolerance)
//    {
//        if (softwarePidEnabled)
//        {
//            if (posPidCtrl != null)
//            {
//                posTolerance = tolerance;
//            }
//            else
//            {
//                throw new IllegalStateException("Software Position PID coefficients have not been set.");
//            }
//        }
//        else
//        {
//            setMotorPositionPidTolerance(tolerance);
//            posTolerance = tolerance;
//        }
//    }   //setPositionPidTolerance
//
//    /**
//     * This method checks if position PID control has reached target.
//     *
//     * @param tolerance specifies the PID tolerance.
//     * @return true if position has reached target, false otherwise.
//     */
//    public boolean getPositionOnTarget(double tolerance)
//    {
//        boolean onTarget;
//
//        if (softwarePidEnabled)
//        {
//            if (posPidCtrl != null)
//            {
//                onTarget = posPidCtrl.isOnTarget(tolerance);
//                posTolerance = tolerance;
//            }
//            else
//            {
//                throw new IllegalStateException("Software Position PID coefficients have not been set.");
//            }
//        }
//        else
//        {
//            onTarget = getMotorPositionOnTarget(tolerance);
//            posTolerance = tolerance;
//        }
//
//        return onTarget;
//    }   //getPositionOnTarget
//
//    /**
//     * This method sets the power compensation callback of the motor's position PID controller.
//     *
//     * @param powerComp specifies the power compensation callback.
//     */
//    public void setPositionPidPowerComp(PowerCompensation powerComp)
//    {
//        if (softwarePidEnabled)
//        {
//            posPowerComp = powerComp;
//        }
//    }   //setPositionPidPowerComp
//
//    /**
//     * This method returns the software position PID controller if one was created by enabling software PID control.
//     * It does not mean software PID control is current enable. It just means software PID control was enabled at
//     * one point. This allows you to configure the software PID controller. For example, whether the target is absolute
//     * or relative, specifying noOscillation etc.
//     *
//     * @return software position PID controller.
//     */
//    public TrcPidController getPositionPidController()
//    {
//        return posPidCtrl;
//    }   //getPositionPidController
//
//    /**
//     * This method sets the PID coefficients of the motor's current PID controller. Note that PID coefficients are
//     * different for software PID and controller built-in PID. If you enable/disable software PID, you need to set
//     * the appropriate PID coefficients accordingly.
//     *
//     * @param pidCoeff specifies the PID coefficients to set.
//     */
//    public void setCurrentPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
//    {
//        if (softwarePidEnabled)
//        {
//            if (currentPidCtrl != null)
//            {
//                currentPidCtrl.setPidCoefficients(pidCoeff);
//            }
//            else
//            {
//                currentPidCtrl = new TrcPidController(instanceName + ".currentPidCtrl", pidCoeff, this::getCurrent);
//                // Set to absolute setpoint because current PID control is generally absolute.
//                currentPidCtrl.setAbsoluteSetPoint(true);
//            }
//        }
//        else
//        {
//            setMotorCurrentPidCoefficients(pidCoeff);
//        }
//    }   //setCurrentPidCoefficients
//
//    /**
//     * This method sets the PID coefficients of the motor's current PID controller. Note that PID coefficients are
//     * different for software PID and controller built-in PID. If you enable/disable software PID, you need to set
//     * the appropriate PID coefficients accordingly.
//     *
//     * @param kP specifies the Kp coefficient.
//     * @param kI specifies the Ki coefficient.
//     * @param kD specifies the Kd coefficient.
//     * @param kF specifies the Kf coefficient.
//     * @param iZone specifies IZone, can be 0.0 if not provided.
//     */
//    public void setCurrentPidCoefficients(double kP, double kI, double kD, double kF, double iZone)
//    {
//        setCurrentPidCoefficients(new TrcPidController.PidCoefficients(kP, kI, kD, kF, iZone));
//    }   //setCurrentPidCoefficients
//
//    /**
//     * This method sets the PID coefficients of the motor's current PID controller. Note that PID coefficients are
//     * different for software PID and controller built-in PID. If you enable/disable software PID, you need to set
//     * the appropriate PID coefficients accordingly.
//     *
//     * @param kP specifies the Kp coefficient.
//     * @param kI specifies the Ki coefficient.
//     * @param kD specifies the Kd coefficient.
//     * @param kF specifies the Kf coefficient.
//     */
//    public void setCurrentPidCoefficients(double kP, double kI, double kD, double kF)
//    {
//        setCurrentPidCoefficients(new TrcPidController.PidCoefficients(kP, kI, kD, kF, 0.0));
//    }   //setCurrentPidCoefficients
//
//    /**
//     * This method returns the PID coefficients of the motor's current PID controller.
//     *
//     * @return PID coefficients of the motor's current PID controller.
//     */
//    public TrcPidController.PidCoefficients getCurrentPidCoefficients()
//    {
//        TrcPidController.PidCoefficients pidCoeff;
//
//        if (softwarePidEnabled)
//        {
//            if (currentPidCtrl != null)
//            {
//                pidCoeff = currentPidCtrl.getPidCoefficients();
//            }
//            else
//            {
//                throw new IllegalStateException("Software Current PID coefficients have not been set.");
//            }
//        }
//        else
//        {
//            pidCoeff = getMotorCurrentPidCoefficients();
//        }
//
//        return pidCoeff;
//    }   //getCurrentPidCoefficients
//
//    /**
//     * This method sets the PID tolerance of the motor's current PID controller.
//     *
//     * @param tolerance specifies the PID tolerance.
//     */
//    public void setCurrentPidTolerance(double tolerance)
//    {
//        if (softwarePidEnabled)
//        {
//            if (currentPidCtrl != null)
//            {
//                currentTolerance = tolerance;
//            }
//            else
//            {
//                throw new IllegalStateException("Software Current PID coefficients have not been set.");
//            }
//        }
//        else
//        {
//            setMotorCurrentPidTolerance(tolerance);
//            currentTolerance = tolerance;
//        }
//    }   //setCurrentPidTolerance
//
//    /**
//     * This method checks if current PID control has reached target.
//     *
//     * @param tolerance specifies the PID tolerance.
//     * @return true if current has reached target, false otherwise.
//     */
//    public boolean getCurrentOnTarget(double tolerance)
//    {
//        boolean onTarget;
//
//        if (softwarePidEnabled)
//        {
//            if (currentPidCtrl != null)
//            {
//                onTarget = currentPidCtrl.isOnTarget(tolerance);
//                currentTolerance = tolerance;
//            }
//            else
//            {
//                throw new IllegalStateException("Software Current PID coefficients have not been set.");
//            }
//        }
//        else
//        {
//            onTarget = getMotorCurrentOnTarget(tolerance);
//            currentTolerance = tolerance;
//        }
//
//        return onTarget;
//    }   //getCurrentOnTarget
//
//    /**
//     * This method sets the power compensation callback of the motor's current PID controller.
//     *
//     * @param powerComp specifies the power compensation callback.
//     */
//    public void setCurrentPidPowerComp(PowerCompensation powerComp)
//    {
//        if (softwarePidEnabled)
//        {
//            currentPowerComp = powerComp;
//        }
//    }   //setCurrentPidPowerComp
//
//    /**
//     * This method returns the software current PID controller if one was created by enabling software PID control.
//     * It does not mean software PID control is current enable. It just means software PID control was enabled at
//     * one point. This allows you to configure the software PID controller. For example, whether the target is absolute
//     * or relative, specifying noOscillation etc.
//     *
//     * @return software current PID controller.
//     */
//    public TrcPidController getCurrentPidController()
//    {
//        return currentPidCtrl;
//    }   //getCurrentPidController
//
//    //
//    // Stall detection.
//    //
//
//    /**
//     * This method performs motor stall detection to protect the motor from burning out. A motor is considered stalled
//     * when at least stallMinPower is applied to the motor and the motor is not turning for at least stallTimeout.
//     * This assumes the caller has acquired the taskParams lock.
//     *
//     * @param power specifies the current motor power.
//     * @return true if the motor is stalled, false otherwise.
//     */
//    private boolean isMotorStalled(double power)
//    {
//        boolean stalled = false;
//
//        if (taskParams.stallMinPower > 0.0 && taskParams.stallTimeout > 0.0)
//        {
//            // Stall protection is ON, check for stall condition.
//            // - power is above stallMinPower
//            // - motor has not moved for at least stallTimeout.
//            double currTime = TrcTimer.getCurrentTime();
//            double currPos = getPosition();
//            if (Math.abs(power) < taskParams.stallMinPower ||
//                Math.abs(currPos - taskParams.prevPos) > taskParams.stallTolerance ||
//                taskParams.prevTime == 0.0)
//            {
//                taskParams.prevPos = currPos;
//                taskParams.prevTime = currTime;
//            }
//
//            if (currTime - taskParams.prevTime > taskParams.stallTimeout)
//            {
//                // We have detected a stalled condition for at least stallTimeout.
//                stalled = true;
//                if (beepDevice != null)
//                {
//                    beepDevice.playTone(beepHighFrequency, beepDuration);
//                }
//                tracer.traceInfo(instanceName, "Stall protection triggered.");
//            }
//        }
//
//        return stalled;
//    }   //isMotorStalled
//
//    /**
//     * This method checks if the motor was stalled and if power has been removed for at least resetTimeout, the
//     * stalled condition is then cleared. This assumes the caller has acquired the taskParams lock.
//     *
//     * @param power specifies the power applying to the motor.
//     * @return true if the motor was stalled, false otherwise.
//     */
//    private boolean resetStall(double power)
//    {
//        boolean wasStalled = taskParams.stalled;
//
//        if (wasStalled)
//        {
//            if (power == 0.0)
//            {
//                // We had a stalled condition but if power is removed for at least reset timeout, we clear the
//                // stalled condition.
//                if (taskParams.resetTimeout == 0.0 ||
//                    TrcTimer.getCurrentTime() - taskParams.prevTime >= taskParams.resetTimeout)
//                {
//                    taskParams.prevPos = getPosition();
//                    taskParams.prevTime = TrcTimer.getCurrentTime();
//                    taskParams.stalled = false;
//                    if (beepDevice != null)
//                    {
//                        beepDevice.playTone(beepLowFrequency, beepDuration);
//                    }
//                }
//            }
//            else
//            {
//                taskParams.prevTime = TrcTimer.getCurrentTime();
//            }
//        }
//
//        return wasStalled;
//    }   //resetStall
//
//    //
//    // Zero calibration.
//    //
//
//    /**
//     * This method is called by the calibration task to zero calibrate a motor. This assumes the caller has acquired
//     * the taskParams lock. This method will update currPower and stalled in taskParams.
//     *
//     * @param calPower specifies the zero calibration power applied to the motor.
//     * @return true if calibration is done, false otherwise.
//     */
//    private boolean zeroCalibratingMotor(double calPower)
//    {
//        boolean done = isLowerLimitSwitchActive() || taskParams.stalled;
//
//        if (done)
//        {
//            // Done with zero calibration of the motor. Call the motor directly to stop, do not call any of
//            // the setPower or setMotorPower because they do not handle zero calibration mode.
//            if (taskParams.stalled)
//            {
//                if (beepDevice != null)
//                {
//                    beepDevice.playTone(beepLowFrequency, beepDuration);
//                }
//                tracer.traceWarn(instanceName, "Stalled, lower limit switch might have failed!");
//            }
//            setControllerMotorPower(0.0, true);
//            resetPosition(false);
//            taskParams.stalled = false;
//        }
//        else
//        {
//            taskParams.stalled = isMotorStalled(calPower);
//            setControllerMotorPower(calPower, true);
//        }
//
//        return done;
//    }   //zeroCalibratingMotor
//
//    //
//    // Close-loop Control Task.
//    //
//
//    /**
//     * This method performs software PID control or monitors completion of motor controller PID control. It is
//     * called periodically to check if PID control is on target. If it is not on target and we are doing software
//     * control, it will calculate the power applying to the motor to get it there. If we reached target in doing
//     * software control and we are holding target, it will maintain target until software control is turned off.
//     * If performing motor controller PID control, this task only monitors progress and sync followers. If it has
//     * reached target, it will signal completion event.
//     *
//     * @param taskType specifies the type of task being run.
//     * @param runMode  specifies the competition mode that is running.
//     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
//     *        false otherwise.
//     */
//    private void pidCtrlTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
//    {
//        TrcEvent completionEvent = null;
//
//        if (pidCtrlTaskPerformanceTimer != null)
//        {
//            pidCtrlTaskPerformanceTimer.recordStartTime();
//        }
//
//        synchronized (taskParams)
//        {
//            boolean onTarget = false;
//            boolean stalled = false;
//            boolean expired = false;
//            Double target = null;
//
//            if (taskParams.calibrating)
//            {
//                // We are in zero calibration mode.
//                if (zeroCalibratingMotor(taskParams.calPower))
//                {
//                    // Done with zero calibration.
//                    taskParams.calibrating = false;
//                    completionEvent = taskParams.notifyEvent;
//                    taskParams.notifyEvent = null;
//                    tracer.traceInfo(instanceName, "Zero calibration done.");
//                }
//            }
//            else
//            {
//                // Do stall detection.
//                taskParams.stalled = isMotorStalled(currMotorPower);
//                if (!resetStall(currMotorPower))
//                {
//                    if (taskParams.currControlMode != ControlMode.Power)
//                    {
//                        // Doing software close loop PID control or monitoring controller PID control.
//                        onTarget =
//                            taskParams.softwarePidCtrl != null?
//                                taskParams.softwarePidCtrl.isOnTarget(taskParams.softwarePidTolerance):
//                            taskParams.currControlMode == ControlMode.Velocity? getMotorVelocityOnTarget(velTolerance):
//                            taskParams.currControlMode == ControlMode.Position? getMotorPositionOnTarget(posTolerance):
//                            taskParams.currControlMode == ControlMode.Current && getMotorCurrentOnTarget(currentTolerance);
//                        stalled = taskParams.softwarePidCtrl != null && taskParams.softwarePidCtrl.isStalled();
//                        expired =           // Only for software PID control.
//                            taskParams.softwarePidCtrl != null && taskParams.timeout != 0.0 &&
//                            TrcTimer.getCurrentTime() >= taskParams.timeout;
//                        boolean doStop =    // Only for software PID control.
//                            taskParams.softwarePidCtrl != null && !taskParams.holdTarget && (onTarget || expired || stalled);
//
//                        if (doStop)
//                        {
//                            // We are stopping motor but control mode is not Power, so don't overwrite it.
//                            setControllerMotorPower(0.0, false);
//                        }
//                        else if (taskParams.softwarePidCtrl == null)
//                        {
//                            // Doing motor controller close loop PID control.
//                            // If a power limit is set, adjust native motor PID control correspondingly.
//                            if (taskParams.powerLimit != null)
//                            {
//                                // Set the same target position but change power limit if necessary.
//                                // If powerLimit did not change from last time, setControllerMotorPosition is a no-op.
//                                setControllerMotorPosition(controllerPosition, taskParams.powerLimit);
//                            }
//                            // We are monitoring for completion and sync the followers if motor controller does not
//                            // support motor following.
//                            synchronized (followingMotorsList)
//                            {
//                                if (!followingMotorsList.isEmpty())
//                                {
//                                    // Get the power of the master motor.
//                                    double power = getPower(false);
//
//                                    for (TrcMotor follower : followingMotorsList)
//                                    {
//                                        switch (taskParams.currControlMode)
//                                        {
//                                            case Velocity:
//                                                // Since this is running in a task loop and if the velocity did not
//                                                // change, we will be setting the same velocity over and over again.
//                                                // So, instead of calling setMotorVelocity, we call
//                                                // setControllerMotorVelocity which has optimization to not sending
//                                                // same velocity if it hasn't change.
//                                                follower.setControllerMotorVelocity(taskParams.motorValue);
//                                                break;
//
//                                            case Position:
//                                                // What does it mean to have position followers?
//                                                // If we are performing position control on followers, the
//                                                // followers must have their own position sensors and they must
//                                                // be synchronized. Even so, it's not guaranteed the movement of
//                                                // the followers are synchronized. Some may move faster than the
//                                                // others. It doesn't make much sense. It only makes sense if the
//                                                // motors are driving the same mechanism and are mechanically
//                                                // linked so you don't need to synchronize them. The motors are
//                                                // just sharing the load. In this case, all the followers should
//                                                // just mimic the power output of the master.
//                                                follower.setControllerMotorPower(power, true);
//                                                break;
//
//                                            case Current:
//                                                // Since this is running in a task loop and if the current did not
//                                                // change, we will be setting the same current over and over again.
//                                                // So, instead of calling setMotorCurrent, we call
//                                                // setControllerMotorCurrent which has optimization to not sending
//                                                // same current if it hasn't change.
//                                                follower.setControllerMotorCurrent(taskParams.motorValue);
//                                                break;
//
//                                            default:
//                                                // If we come here, it's power control mode which we excluded from
//                                                // the above code. So we should never come here.
//                                                throw new IllegalStateException("Should never come here.");
//                                        }
//                                    }
//                                }
//                            }
//                        }
//                        else
//                        {
//                            // Doing software PID control.
//                            // We are either holding target or we are not yet onTarget or stalled or timed out,
//                            // keep applying PID calculated power.
//                            double power = taskParams.softwarePidCtrl.getOutput();
//
//                            if (taskParams.powerLimit != null)
//                            {
//                                // Apply power limit to the calculated PID power.
//                                // Only applicable for Position control mode.
//                                power = TrcUtil.clipRange(power, taskParams.powerLimit);
//                            }
//
//                            if (taskParams.powerComp != null)
//                            {
//                                power = TrcUtil.clipRange(power + taskParams.powerComp.getCompensation(power));
//                            }
//                            // Software PID control sets motor power but control mode is not Power, so don't
//                            // overwrite it.
//                            setControllerMotorPower(power, false);
//
//                            tracer.traceDebug(
//                                instanceName, "onTarget=%s(%f/%f), expired=%s, stalled=%s, powerLimit=%s, power=%f",
//                                onTarget, getPosition(), taskParams.softwarePidCtrl.getTarget(), expired, stalled,
//                                taskParams.powerLimit, power);
//                            if (tracePidInfo)
//                            {
//                                taskParams.softwarePidCtrl.printPidInfo(tracer, verbosePidInfo, battery);
//                            }
//                        }
//
//                        if (onTarget || stalled || expired)
//                        {
//                            taskParams.softwarePidCtrl.endStallDetection();
//                            completionEvent = taskParams.notifyEvent;
//                            target = closeLoopControlTarget;
//                            taskParams.notifyEvent = null;
//                            closeLoopControlTarget = null;
//                        }
//                    }
//                }
//            }
//
//            if (pidCtrlTaskPerformanceTimer != null)
//            {
//                pidCtrlTaskPerformanceTimer.recordEndTime();
//            }
//
//            if (completionEvent != null)
//            {
//                double currValue;
//
//                completionEvent.signal();
//                // Only print this once to reduce clutter.
//                switch (taskParams.currControlMode)
//                {
//                    case Velocity:
//                        currValue = getVelocity();
//                        break;
//                    case Position:
//                        currValue = getPosition();
//                        break;
//                    case Current:
//                        currValue = getCurrent();
//                        break;
//                    default:
//                        // Should never come here but do it to make compiler happy.
//                        currValue = getPower();
//                        break;
//                }
//                tracer.traceInfo(
//                    instanceName, "onTarget=%s(%f/%s), stalled=%s, expired=%s, event=%s",
//                    onTarget, currValue, target, stalled, expired, completionEvent);
//            }
//        }
//    }   //pidCtrlTask
//
//    //
//    // Reset position on digital trigger support.
//    //
//
//    /**
//     * This method creates a digital trigger on the motor's lower limit switch. It resets the position sensor
//     * reading when the digital input is triggered. This is intended to be called as part of motor initialization.
//     * Therefore, it is not designed to be ownership-aware.
//     *
//     * @param triggerMode specifies the trigger mode.
//     * @param triggerCallback specifies an event callback if the trigger occurred, null if none specified.
//     */
//    public void resetPositionOnLowerLimitSwitch(TriggerMode triggerMode, TrcEvent.Callback triggerCallback)
//    {
//        tracer.traceDebug(instanceName, "triggerMode=" + triggerMode + ",callback=" + (triggerCallback != null));
//        digitalTrigger = new TrcTriggerDigitalInput(
//            instanceName + ".digitalTrigger", new TrcMotorLimitSwitch(instanceName + ".lowerLimit", this, false));
//        this.triggerMode = triggerMode;
//        this.triggerCallback = triggerCallback;
//
//        if (triggerCallback != null)
//        {
//            triggerCallbackContext = new AtomicBoolean();
//            triggerCallbackEvent = new TrcEvent(instanceName + ".triggerCallbackEvent");
//        }
//
//        digitalTrigger.enableTrigger(this::resetTriggerCallback);
//    }   //resetPositionOnLowerLimitSwitch
//
//    /**
//     * This method creates a digital trigger on the motor's lower limit switch. It resets the position sensor
//     * reading when the digital input is triggered. This is intended to be called as part of motor initialization.
//     * Therefore, it is not designed to be ownership-aware.
//     *
//     * @param triggerMode specifies the trigger mode.
//     */
//    public void resetPositionOnLowerLimitSwitch(TriggerMode triggerMode)
//    {
//        resetPositionOnLowerLimitSwitch(triggerMode, null);
//    }   //resetPositionOnLowerLimitSwitch
//
//    /**
//     * This method creates a digital trigger on the motor's lower limit switch. It resets the position sensor
//     * reading when the digital input is triggered. This is intended to be called as part of motor initialization.
//     * Therefore, it is not designed to be ownership-aware.
//     *
//     * @param triggerCallback specifies an event callback if the trigger occurred, null if none specified.
//     */
//    public void resetPositionOnLowerLimitSwitch(TrcEvent.Callback triggerCallback)
//    {
//        resetPositionOnLowerLimitSwitch(TriggerMode.OnActive, triggerCallback);
//    }   //resetPositionOnLowerLimitSwitch
//
//    /**
//     * This method creates a digital trigger on the motor's lower limit switch. It resets the position sensor
//     * reading when the digital input is triggered. This is intended to be called as part of motor initialization.
//     * Therefore, it is not designed to be ownership-aware.
//     */
//    public void resetPositionOnLowerLimitSwitch()
//    {
//        resetPositionOnLowerLimitSwitch(TriggerMode.OnActive, null);
//    }   //resetPositionOnLowerLimitSwitch
//
//    /**
//     * This method is called when the digital input device has changed state.
//     *
//     * @param context specifies true if the digital device state is active, false otherwise.
//     */
//    private void resetTriggerCallback(Object context)
//    {
//        boolean active = ((AtomicBoolean) context).get();
//
//        tracer.traceDebug(instanceName, "trigger=" + digitalTrigger + ",active=" + active);
//        if (triggerMode == TriggerMode.OnBoth ||
//            triggerMode == TriggerMode.OnActive && active ||
//            triggerMode == TriggerMode.OnInactive && !active)
//        {
//            tracer.traceInfo(instanceName, "Reset position on digital trigger! (BeforePos=%f)", getPosition());
//            resetPosition(false);
//        }
//
//        if (triggerCallbackEvent != null)
//        {
//            triggerCallbackContext.set(active);
//            triggerCallbackEvent.setCallback(triggerCallback, triggerCallbackContext);
//            triggerCallbackEvent.signal();
//        }
//    }   //resetTriggerCallback
//
//    /**
//     * This method checks if the PID motor is in the middle of zero calibration.
//     *
//     * @return true if zero calibrating, false otherwise.
//     */
//    public boolean isCalibrating()
//    {
//        return taskParams.calibrating;
//    }   //isCalibrating
//
//    /**
//     * This method starts zero calibration mode by moving the motor with specified calibration power until a limit
//     * switch is hit or the motor is stalled.
//     * Generally, calibration power should be negative so that the motor will move towards the lower limit switch.
//     * However, in some scenarios such as a turret that can turn all the way around and has only one limit switch,
//     * it may be necessary to use a positive calibration power to move it towards the limit switch instead of using
//     * negative calibration power and turning the long way around that may cause wires to entangle.
//     * If this method specifies an owner and the subsystem was not owned by it, it will acquire exclusive ownership
//     * on its behalf and release ownership after the operation is completed.
//     *
//     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
//     * @param calPower specifies the motor power for the zero calibration, can be positive or negative depending on
//     *        the desire direction of movement.
//     * @param completionEvent specifies an event to signal when zero calibration is done, can be null if not provided.
//     * @param callback specifies a callback handler when zero calibration is done.
//     */
//    private void zeroCalibrate(String owner, double calPower, TrcEvent completionEvent, TrcEvent.Callback callback)
//    {
//        if (completionEvent != null)
//        {
//            completionEvent.clear();
//        }
//
//        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, tracer);
//        if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;
//
//        if (validateOwnership(owner))
//        {
//            // Stop previous operation if there is one.
//            stop();
//
//            synchronized (taskParams)
//            {
//                taskParams.calPower = calPower;
//                taskParams.notifyEvent = completionEvent;
//                taskParams.calibrating = true;
//                taskParams.prevTime = TrcTimer.getCurrentTime();
//            }
//        }
//    }   //zeroCalibrate
//
//    /**
//     * This method starts zero calibration mode by moving the motor with specified calibration power until a limit
//     * switch is hit or the motor is stalled.
//     * Generally, calibration power should be negative so that the motor will move towards the lower limit switch.
//     * However, in some scenarios such as a turret that can turn all the way around and has only one limit switch,
//     * it may be necessary to use a positive calibration power to move it towards the limit switch instead of using
//     * negative calibration power and turning the long way around that may cause wires to entangle.
//     * If this method specifies an owner and the subsystem was not owned by it, it will acquire exclusive ownership
//     * on its behalf and release ownership after the operation is completed.
//     *
//     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
//     * @param calPower specifies the motor power for the zero calibration, can be positive or negative depending on
//     *        the desire direction of movement.
//     * @param completionEvent specifies an event to signal when zero calibration is done, can be null if not provided.
//     */
//    public void zeroCalibrate(String owner, double calPower, TrcEvent completionEvent)
//    {
//        zeroCalibrate(owner, calPower, completionEvent, null);
//    }   //zeroCalibrate
//
//    /**
//     * This method starts zero calibration mode by moving the motor with specified calibration power until a limit
//     * switch is hit or the motor is stalled.
//     * Generally, calibration power should be negative so that the motor will move towards the lower limit switch.
//     * However, in some scenarios such as a turret that can turn all the way around and has only one limit switch,
//     * it may be necessary to use a positive calibration power to move it towards the limit switch instead of using
//     * negative calibration power and turning the long way around that may cause wires to entangle.
//     *
//     * @param calPower specifies the motor power for the zero calibration, can be positive or negative depending on
//     *        the desire direction of movement.
//     * @param completionEvent specifies an event to signal when zero calibration is done, can be null if not provided.
//     */
//    public void zeroCalibrate(double calPower, TrcEvent completionEvent)
//    {
//        zeroCalibrate(null, calPower, completionEvent, null);
//    }   //zeroCalibrate
//
//    /**
//     * This method starts zero calibration mode by moving the motor with specified calibration power until a limit
//     * switch is hit or the motor is stalled.
//     * Generally, calibration power should be negative so that the motor will move towards the lower limit switch.
//     * However, in some scenarios such as a turret that can turn all the way around and has only one limit switch,
//     * it may be necessary to use a positive calibration power to move it towards the limit switch instead of using
//     * negative calibration power and turning the long way around that may cause wires to entangle.
//     * If this method specifies an owner and the subsystem was not owned by it, it will acquire exclusive ownership
//     * on its behalf and release ownership after the operation is completed.
//     *
//     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
//     * @param calPower specifies the motor power for the zero calibration, can be positive or negative depending on
//     *        the desire direction of movement.
//     * @param callback specifies a callback handler when zero calibration is done.
//     */
//    public void zeroCalibrate(String owner, double calPower, TrcEvent.Callback callback)
//    {
//        zeroCalibrate(owner, calPower, null, callback);
//    }   //zeroCalibrate
//
//    /**
//     * This method starts zero calibration mode by moving the motor with specified calibration power until a limit
//     * switch is hit or the motor is stalled.
//     * Generally, calibration power should be negative so that the motor will move towards the lower limit switch.
//     * However, in some scenarios such as a turret that can turn all the way around and has only one limit switch,
//     * it may be necessary to use a positive calibration power to move it towards the limit switch instead of using
//     * negative calibration power and turning the long way around that may cause wires to entangle.
//     *
//     * @param calPower specifies the motor power for the zero calibration, can be positive or negative depending on
//     *        the desire direction of movement.
//     * @param callback specifies a callback handler when zero calibration is done.
//     */
//    public void zeroCalibrate(double calPower, TrcEvent.Callback callback)
//    {
//        zeroCalibrate(null, calPower, null, callback);
//    }   //zeroCalibrate
//
//    /**
//     * This method starts zero calibration mode by moving the motor with specified calibration power until a limit
//     * switch is hit or the motor is stalled.
//     * Generally, calibration power should be negative so that the motor will move towards the lower limit switch.
//     * However, in some scenarios such as a turret that can turn all the way around and has only one limit switch,
//     * it may be necessary to use a positive calibration power to move it towards the limit switch instead of using
//     * negative calibration power and turning the long way around that may cause wires to entangle.
//     *
//     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
//     * @param calPower specifies the motor power for the zero calibration, can be positive or negative depending on
//     *        the desire direction of movement.
//     */
//    public void zeroCalibrate(String owner, double calPower)
//    {
//        zeroCalibrate(owner, calPower, null, null);
//    }   //zeroCalibrate
//
//    /**
//     * This method starts zero calibration mode by moving the motor with specified calibration power until a limit
//     * switch is hit or the motor is stalled.
//     * Generally, calibration power should be negative so that the motor will move towards the lower limit switch.
//     * However, in some scenarios such as a turret that can turn all the way around and has only one limit switch,
//     * it may be necessary to use a positive calibration power to move it towards the limit switch instead of using
//     * negative calibration power and turning the long way around that may cause wires to entangle.
//     *
//     * @param calPower specifies the motor power for the zero calibration, can be positive or negative depending on
//     *        the desire direction of movement.
//     */
//    public void zeroCalibrate(double calPower)
//    {
//        zeroCalibrate(null, calPower, null, null);
//    }   //zeroCalibrate
//
//    //
//    // Position presets.
//    //
//
//    /**
//     * This method sets an array of preset positions for the motor.
//     *
//     * @param tolerance specifies the preset tolerance.
//     * @param posPresets specifies an array of preset positions in scaled unit.
//     */
//    public void setPosPresets(double tolerance, double... posPresets)
//    {
//        this.presetTolerance = tolerance;
//        this.posPresets = posPresets;
//    }   //setPosPresets
//
//    /**
//     * This method checks if the preset index is within the preset table.
//     *
//     * @param index specifies the preset table index to check.
//     * @return true if there is a preset table and the index is within the table.
//     */
//    public boolean validatePresetIndex(int index)
//    {
//        return posPresets != null && index >= 0 && index < posPresets.length;
//    }   //validatePresetIndex
//
//    /**
//     * This method returns the preset position at the specified index.
//     *
//     * @param index specifies the index into the preset position table.
//     * @return preset position.
//     */
//    public double getPresetPosition(int index)
//    {
//        return posPresets[index];
//    }   //getPresetPosition
//
//    /**
//     * This method sets the motor to the specified preset position.
//     *
//     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
//     * @param delay specifies delay time in seconds before setting position, can be zero if no delay.
//     * @param presetIndex specifies the index to the preset position array.
//     * @param holdTarget specifies true to hold target after PID operation is completed.
//     * @param powerLimit specifies the maximum power limit.
//     * @param event specifies the event to signal when done, can be null if not provided.
//     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
//     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
//     *                specified, it should be set to zero.
//     */
//    public void setPresetPosition(
//        String owner, double delay, int presetIndex, boolean holdTarget, double powerLimit, TrcEvent event,
//        double timeout)
//    {
//        if (validatePresetIndex(presetIndex))
//        {
//            setPosition(owner, delay, posPresets[presetIndex], holdTarget, powerLimit, event, timeout);
//        }
//    }   //setPresetPosition
//
//    /**
//     * This method sets the motor to the specified preset position.
//     *
//     * @param delay specifies delay time in seconds before setting position, can be zero if no delay.
//     * @param preset specifies the index to the preset position array.
//     * @param holdTarget specifies true to hold target after PID operation is completed.
//     * @param powerLimit specifies the maximum power limit.
//     * @param event specifies the event to signal when done, can be null if not provided.
//     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
//     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
//     *                specified, it should be set to zero.
//     */
//    public void setPresetPosition(
//        double delay, int preset, boolean holdTarget, double powerLimit, TrcEvent event, double timeout)
//    {
//        setPresetPosition(null, delay, preset, holdTarget, powerLimit, event, timeout);
//    }   //setPresetPosition
//
//    /**
//     * This method sets the motor to the specified preset position.
//     *
//     * @param preset specifies the index to the preset position array.
//     * @param holdTarget specifies true to hold target after PID operation is completed.
//     * @param powerLimit specifies the maximum power limit.
//     * @param event specifies the event to signal when done, can be null if not provided.
//     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
//     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
//     *                specified, it should be set to zero.
//     */
//    public void setPresetPosition(int preset, boolean holdTarget, double powerLimit, TrcEvent event, double timeout)
//    {
//        setPresetPosition(null, 0.0, preset, holdTarget, powerLimit, event, timeout);
//    }   //setPresetPosition
//
//    /**
//     * This method sets the motor to the specified preset position.
//     *
//     * @param preset specifies the index to the preset position array.
//     * @param powerLimit specifies the maximum power limit.
//     * @param event specifies the event to signal when done, can be null if not provided.
//     */
//    public void setPresetPosition(int preset, double powerLimit, TrcEvent event)
//    {
//        setPresetPosition(null, 0.0, preset, true, powerLimit, event, 0.0);
//    }   //setPresetPosition
//
//    /**
//     * This method sets the motor to the specified preset position.
//     *
//     * @param preset specifies the index to the preset position array.
//     * @param event specifies the event to signal when done, can be null if not provided.
//     */
//    public void setPresetPosition(int preset, TrcEvent event)
//    {
//        setPresetPosition(null, 0.0, preset, true, 1.0, event, 0.0);
//    }   //setPresetPosition
//
//    /**
//     * This method sets the motor to the specified preset position.
//     *
//     * @param delay specifies delay time in seconds before setting position, can be zero if no delay.
//     * @param preset specifies the index to the preset position array.
//     * @param powerLimit specifies the maximum power limit.
//     */
//    public void setPresetPosition(double delay, int preset, double powerLimit)
//    {
//        setPresetPosition(null, delay, preset, true, powerLimit, null, 0.0);
//    }   //setPresetPosition
//
//    /**
//     * This method sets the motor to the specified preset position.
//     *
//     * @param delay specifies delay time in seconds before setting position, can be zero if no delay.
//     * @param preset specifies the index to the preset position array.
//     */
//    public void setPresetPosition(double delay, int preset)
//    {
//        setPresetPosition(null, delay, preset, true, 1.0, null, 0.0);
//    }   //setPresetPosition
//
//    /**
//     * This method sets the motor to the specified preset position.
//     *
//     * @param preset specifies the index to the preset position array.
//     * @param powerLimit specifies the maximum power limit.
//     */
//    public void setPresetPosition(int preset, double powerLimit)
//    {
//        setPresetPosition(null, 0.0, preset, true, powerLimit, null, 0.0);
//    }   //setPresetPosition
//
//    /**
//     * This method sets the motor to the specified preset position.
//     *
//     * @param preset specifies the index to the preset position array.
//     */
//    public void setPresetPosition(int preset)
//    {
//        setPresetPosition(null, 0.0, preset, true, 1.0, null, 0.0);
//    }   //setPresetPosition
//
//    /**
//     * This method determines the next preset index up from the current position.
//     *
//     * @return next preset index up, -1 if there is no preset table.
//     */
//    public int nextPresetIndexUp()
//    {
//        int index = -1;
//
//        if (posPresets != null)
//        {
//            double currPos = getPosition() + presetTolerance;
//
//            for (int i = 0; i < posPresets.length; i++)
//            {
//                if (posPresets[i] > currPos )
//                {
//                    index = i;
//                    break;
//                }
//            }
//
//            if (index == -1)
//            {
//                index = posPresets.length - 1;
//            }
//        }
//
//        return index;
//    }   //nextPresetIndexUp
//
//    /**
//     * This method determines the next preset index down from the current position.
//     *
//     * @return next preset index down, -1 if there is no preset table.
//     */
//    public int nextPresetIndexDown()
//    {
//        int index = -1;
//
//        if (posPresets != null)
//        {
//            double currPos = getPosition() - presetTolerance;
//
//            for (int i = posPresets.length - 1; i >= 0; i--)
//            {
//                if (posPresets[i] < currPos)
//                {
//                    index = i;
//                    break;
//                }
//            }
//
//            if (index == -1)
//            {
//                index = 0;
//            }
//        }
//
//        return index;
//    }   //nextPresetIndexDown
//
//    /**
//     * This method sets the motor to the next preset position up or down from the current position.
//     *
//     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
//     *        automatically release ownership when the motor movement is completed, can be null if no ownership
//     *        is required.
//     * @param presetUp specifies true to move to next preset up, false to move to next preset down.
//     * @param powerLimit specifies the maximum power limit.
//     */
//    private void setNextPresetPosition(String owner, boolean presetUp, double powerLimit)
//    {
//        int index = presetUp? nextPresetIndexUp(): nextPresetIndexDown();
//
//        if (index != -1)
//        {
//            setPresetPosition(owner, 0.0, index, true, powerLimit, null, 0.0);
//        }
//    }   //setNextPresetPosition
//
//    /**
//     * This method sets the motor to the next preset position up from the current position.
//     *
//     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
//     *        automatically release ownership when the motor movement is completed, can be null if no ownership
//     *        is required.
//     * @param powerLimit specifies the maximum power limit.
//     */
//    public void presetPositionUp(String owner, double powerLimit)
//    {
//        setNextPresetPosition(owner, true, powerLimit);
//    }   //presetPositionUp
//
//    /**
//     * This method sets the motor to the next preset position down from the current position.
//     *
//     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
//     *        automatically release ownership when the motor movement is completed, can be null if no ownership
//     *        is required.
//     * @param powerLimit specifies the maximum power limit.
//     */
//    public void presetPositionDown(String owner, double powerLimit)
//    {
//        setNextPresetPosition(owner, false, powerLimit);
//    }   //presetPositionDown
//
//    //
//    // Odometry.
//    //
//
//    /**
//     * This method clears the list of motors that register for odometry monitoring. This method should only be called
//     * by the task scheduler.
//     *
//     * @param removeOdometryTask specifies true to also remove the odometry task object, false to leave it alone.
//     *        This is mainly for FTC, FRC should always set this to false.
//     */
//    public static void clearOdometryMotorsList(boolean removeOdometryTask)
//    {
//        synchronized (odometryMotors)
//        {
//            if (odometryMotors.size() > 0)
//            {
//                odometryMotors.clear();
//                odometryTaskObj.unregisterTask();
//            }
//            //
//            // We must clear the task object because FTC opmode stuck around even after it has ended. So the task
//            // object would have a stale odometryTask if we run the opmode again.
//            //
//            if (removeOdometryTask)
//            {
//                odometryTaskObj = null;
//            }
//        }
//    }   //clearOdometryMotorsList
//
//    /**
//     * This method returns the number of motors in the list registered for odometry monitoring.
//     *
//     * @return number of motors in the list.
//     */
//    public static int getNumOdometryMotors()
//    {
//        int numMotors;
//
//        synchronized (odometryMotors)
//        {
//            numMotors = odometryMotors.size();
//        }
//
//        return numMotors;
//    }   //getNumOdometryMotors
//
//    /**
//     * This method enables/disables the task that monitors the motor odometry. Since odometry task takes up CPU cycle,
//     * it should not be enabled if the user doesn't need motor odometry info.
//     *
//     * @param enabled specifies true to enable odometry task, disable otherwise.
//     * @param resetOdometry specifies true to reset odometry, false otherwise.
//     * @param resetHardware specifies true to reset odometry hardware, false otherwise. This is only applicable when
//     *        enabling odometry, not used when disabling.
//     */
//    public void setOdometryEnabled(boolean enabled, boolean resetOdometry, boolean resetHardware)
//    {
//        tracer.traceDebug(
//            instanceName, "enabled=" + enabled + ", resetOd=" + resetOdometry + ", hwReset=" + resetHardware);
//        if (enabled)
//        {
//            if (resetOdometry)
//            {
//                resetOdometry(resetHardware);
//            }
//
//            synchronized (odometryMotors)
//            {
//                //
//                // Add only if this motor is not already on the list.
//                //
//                if (!odometryMotors.contains(this))
//                {
//                    odometryMotors.add(this);
//                    if (odometryMotors.size() == 1)
//                    {
//                        //
//                        // We are the first one on the list, start the task.
//                        //
//                        odometryTaskObj.registerTask(TaskType.STANDALONE_TASK);
//                    }
//                }
//            }
//        }
//        else
//        {
//            synchronized (odometryMotors)
//            {
//                odometryMotors.remove(this);
//                if (odometryMotors.isEmpty())
//                {
//                    //
//                    // We were the only one on the list, stop the task.
//                    //
//                    odometryTaskObj.unregisterTask();
//                }
//            }
//        }
//        odometryEnabled = enabled;
//    }   //setOdometryEnabled
//
//    /**
//     * This method checks if the odometry of this motor is enabled.
//     *
//     * @return true if odometry of this motor is enabled, false if disabled.
//     */
//    public boolean isOdometryEnabled()
//    {
//        return odometryEnabled;
//    }   //isOdometryEnabled
//
//    //
//    // Implements TrcOdometrySensor interfaces.
//    //
//
//    /**
//     * This method returns the instance name.
//     *
//     * @return instance name.
//     */
//    @Override
//    public String getName()
//    {
//        return instanceName;
//    }   //getName
//
//    /**
//     * This method resets the odometry data and sensor.
//     *
//     * @param resetHardware specifies true to do a hardware reset, false to do a software reset. Hardware reset may
//     *        require some time to complete and will block this method from returning until finish.
//     */
//    @Override
//    public void resetOdometry(boolean resetHardware)
//    {
//        resetPosition(resetHardware);
//        synchronized (odometry)
//        {
//            odometry.prevTimestamp = odometry.currTimestamp = TrcTimer.getCurrentTime();
//            odometry.prevPos = odometry.currPos = 0.0;
//            odometry.velocity = 0.0;
//        }
//    }   //resetOdometry
//
//    /**
//     * This method returns a copy of the odometry data of the specified axis. It must be a copy so it won't change while
//     * the caller is accessing the data fields.
//     *
//     * @param axisIndex specifies the axis index if it is a multi-axes sensor, 0 if it is a single axis sensor (not used).
//     * @return a copy of the odometry data of the specified axis.
//     */
//    @Override
//    public Odometry getOdometry(int axisIndex)
//    {
//        Odometry odom;
//
//        synchronized (odometry)
//        {
//            if (!odometryEnabled)
//            {
//                // Odometry is disabled, returned the last update.
//                tracer.traceErr(instanceName, "Motor odometry is not enabled.");
//            }
//
//            odom = odometry.clone();
//        }
//
//        return odom;
//    }   //getOdometry
//
//    /**
//     * This method is called periodically to update motor odometry data. Odometry data includes position and velocity
//     * data. By using this task to update odometry at a periodic rate, it allows robot code to obtain odometry data
//     * from the cached data maintained by this task instead of repeatedly reading it directly from the motor
//     * controller which may impact performance because it may involve initiating USB/CAN/I2C bus cycles. So even
//     * though some motor controller hardware may keep track of its own velocity, it may be beneficial to just let the
//     * odometry task to calculate it.
//     *
//     * @param taskType specifies the type of task being run.
//     * @param runMode  specifies the competition mode that is running.
//     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
//     *        false otherwise.
//     */
//    private static void odometryTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
//    {
//        synchronized (odometryMotors)
//        {
//            for (TrcMotor motor : odometryMotors)
//            {
//                synchronized (motor.odometry)
//                {
//                    motor.odometry.prevTimestamp = motor.odometry.currTimestamp;
//                    motor.odometry.prevPos = motor.odometry.currPos;
//                    motor.odometry.currTimestamp = TrcTimer.getCurrentTime();
//                    motor.odometry.currPos = motor.getControllerPosition(true);
//                    //
//                    // Detect spurious encoder reading.
//                    //
//                    double low = Math.abs(motor.odometry.prevPos);
//                    double high = Math.abs(motor.odometry.currPos);
//
//                    if (low > high)
//                    {
//                        double temp = high;
//                        high = low;
//                        low = temp;
//                    }
//                    // To be spurious, motor must jump 10000+ units, and change by 8+ orders of magnitude
//                    // log10(high)-log10(low) gives change in order of magnitude
//                    // use log rules, equal to log10(high/low) >= 8
//                    // change of base, log2(high/low)/log2(10) >= 8
//                    // log2(high/low) >= 26.6ish
//                    // Math.getExponent() is equal to floor(log2())
//                    if (high - low > 10000)
//                    {
//                        low = Math.max(low, 1);
//                        if (Math.getExponent(high / low) >= 27)
//                        {
//                            TrcDbgTrace.globalTraceWarn(
//                                motor.instanceName, "WARNING-Spurious encoder detected! odometry=" + motor.odometry);
//                            // Throw away spurious data and use previous data instead.
//                            motor.odometry.currPos = motor.odometry.prevPos;
//                        }
//                    }
//
//                    try
//                    {
//                        motor.odometry.velocity = motor.getMotorVelocity();
//                    }
//                    catch (UnsupportedOperationException e)
//                    {
//                        // It doesn't support velocity data so calculate it ourselves.
//                        double timeDelta = motor.odometry.currTimestamp - motor.odometry.prevTimestamp;
//                        motor.odometry.velocity =
//                            timeDelta == 0.0 ? 0.0 : (motor.odometry.currPos - motor.odometry.prevPos) / timeDelta;
//                    }
//                    TrcDbgTrace.globalTraceDebug(motor.instanceName, "Odometry=" + motor.odometry);
//                }
//            }
//        }
//    }   //odometryTask
//
//    /**
//     * This method is called before the runMode is about to stop so we can disable odometry.
//     *
//     * @param taskType specifies the type of task being run.
//     * @param runMode  specifies the competition mode that is running.
//     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
//     *        false otherwise.
//     */
//    private void odometryCleanupTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
//    {
//        clearOdometryMotorsList(false);
//    }   //odometryCleanupTask
//
//    //
//    // Performance monitoring.
//    //
//
//    /**
//     * This method enables/disables the elapsed timers for performance monitoring.
//     *
//     * @param enabled specifies true to enable elapsed timers, false to disable.
//     */
//    public static void setElapsedTimerEnabled(boolean enabled)
//    {
//        if (enabled)
//        {
//            if (motorGetPositionElapsedTimer == null)
//            {
//                motorGetPositionElapsedTimer = new TrcElapsedTimer(moduleName + ".getPos", 2.0);
//            }
//
//            if (motorSetPowerElapsedTimer == null)
//            {
//                motorSetPowerElapsedTimer = new TrcElapsedTimer(moduleName + ".setPower", 2.0);
//            }
//
//            if (motorSetVelocityElapsedTimer == null)
//            {
//                motorSetVelocityElapsedTimer = new TrcElapsedTimer(moduleName + ".setVel", 2.0);
//            }
//
//            if (motorSetPositionElapsedTimer == null)
//            {
//                motorSetPositionElapsedTimer = new TrcElapsedTimer(moduleName + ".setPos", 2.0);
//            }
//
//            if (motorSetCurrentElapsedTimer == null)
//            {
//                motorSetCurrentElapsedTimer = new TrcElapsedTimer(moduleName + ".setCurrent", 2.0);
//            }
//        }
//        else
//        {
//            motorGetPositionElapsedTimer = null;
//            motorSetPowerElapsedTimer = null;
//            motorSetVelocityElapsedTimer = null;
//            motorSetPositionElapsedTimer = null;
//            motorSetCurrentElapsedTimer = null;
//        }
//    }   //setElapsedTimerEnabled
//
//    /**
//     * This method prints the elapsed time info using the given tracer.
//     *
//     * @param tracer specifies the tracer to be used to print the info.
//     */
//    public static void printElapsedTime(TrcDbgTrace tracer)
//    {
//        if (motorGetPositionElapsedTimer != null)
//        {
//            motorGetPositionElapsedTimer.printElapsedTime(tracer);
//        }
//
//        if (motorSetPowerElapsedTimer != null)
//        {
//            motorSetPowerElapsedTimer.printElapsedTime(tracer);
//        }
//
//        if (motorSetVelocityElapsedTimer != null)
//        {
//            motorSetVelocityElapsedTimer.printElapsedTime(tracer);
//        }
//
//        if (motorSetPositionElapsedTimer != null)
//        {
//            motorSetPositionElapsedTimer.printElapsedTime(tracer);
//        }
//
//        if (motorSetCurrentElapsedTimer != null)
//        {
//            motorSetCurrentElapsedTimer.printElapsedTime(tracer);
//        }
//    }   //printElapsedTime
//
//    /**
//     * This method prints the PID control task performance info.
//     */
//    public void printPidControlTaskPerformance()
//    {
//        if (pidCtrlTaskPerformanceTimer != null)
//        {
//            tracer.traceInfo(instanceName, pidCtrlTaskPerformanceTimer.toString());
//        }
//    }   //printPidControlTaskPerformance
//
////    /**
////     * Transforms the desired percentage of motor stall torque to the motor duty cycle (aka power)
////     * that would give us that amount of torque at the current motor speed.
////     *
////     * @param desiredStallTorquePercentage specifies the desired percentage of motor torque to receive in percent of
////     *        motor stall torque.
////     * @return power percentage to apply to the motor to generate the desired torque (to the best ability of the motor).
////     */
////    private double transformTorqueToMotorPower(double desiredStallTorquePercentage)
////    {
////        double power;
////        //
////        // Leverage motor curve information to linearize torque output across varying RPM
////        // as best we can. We know that max torque is available at 0 RPM and zero torque is
////        // available at max RPM. Use that relationship to proportionately boost voltage output
////        // as motor speed increases.
////        //
////        final double currSpeedSensorUnitPerSec = Math.abs(getVelocity());
////        final double currNormalizedSpeed = currSpeedSensorUnitPerSec / maxMotorVelocity;
////
////        // Max torque percentage declines proportionally to motor speed.
////        final double percentMaxTorqueAvailable = 1 - currNormalizedSpeed;
////
////        if (percentMaxTorqueAvailable > 0)
////        {
////            power = desiredStallTorquePercentage / percentMaxTorqueAvailable;
////        }
////        else
////        {
////            // When we exceed max motor speed (and the correction factor is undefined), apply 100% voltage.
////            power = Math.signum(desiredStallTorquePercentage);
////        }
////        tracer.traceDebug(instanceName, "torque=%f, power=%f", desiredStallTorquePercentage, power);
////
////        return power;
////    }   //transformTorqueToMotorPower
//
//}   //class TrcMotor