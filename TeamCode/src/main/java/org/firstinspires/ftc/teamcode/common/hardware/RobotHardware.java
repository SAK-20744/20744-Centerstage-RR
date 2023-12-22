package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.function.Inverse;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile.ProfileConstraints;
//import org.firstinspires.ftc.teamcode.common.util.InverseKinematics;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WServo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.annotation.Nonnegative;

@Config
public class RobotHardware {

    //Drivetrain
    public DcMotorEx dtFrontLeftMotor;
    public DcMotorEx dtFrontRightMotor;
    public DcMotorEx dtBackLeftMotor;
    public DcMotorEx dtBackRightMotor;

    //Arm 1
    public DcMotor left_lift;
    public DcMotor right_lift;
    public WEncoder leftLiftEncoder;
    public WEncoder rightLiftEncoder;
    public WActuatorGroup leftLiftActuator;
    public WActuatorGroup rightLiftActuator;

    //Arm2
    public CRServo leftArm;
    public CRServo rightArm;
    public AnalogInput leftAnalogInput;
    public AnalogInput rightAnalogInput;
    public AbsoluteAnalogEncoder leftArmEncoder;
    public AbsoluteAnalogEncoder rightArmEncoder;
    public WActuatorGroup leftArmActuator;
    public WActuatorGroup rightArmActuator;

    // Intake
    public CRServo intake;
    public Servo door;
    public Servo wrist;

    // Plane Launcher
    public Servo plane;

    // Odometry Pod Encoders
//    public WEncoder podLeft;
//    public WEncoder podRight;
//    public WEncoder podFront;

    //HardwareMap storage
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

//     * Voltage timer and voltage value
    private ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage = 12.0;

    /**
     * Singleton variables.
     */
    private static RobotHardware instance = null;
    public boolean enabled;

    private IMU imu;
    public List<LynxModule> modules;

    private ArrayList<WSubsystem> subsystems;

    private double imuAngle ;

    /**
     * Creating the singleton the first time, instantiating.
     */
    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    /**
     * Created at the start of every OpMode.
     *
     * @param hardwareMap The HardwareMap of the robot, storing all hardware devices
     * @param telemetry Saved for later in the event FTC Dashboard used
     */
    public void init(final HardwareMap hardwareMap, final Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        if (Globals.USING_DASHBOARD) {
            this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        } else {
            this.telemetry = telemetry;
        }

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        this.subsystems = new ArrayList<>();

        modules = hardwareMap.getAll(LynxModule.class);
        modules.get(0).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        modules.get(1).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // DRIVETRAIN
        this.dtBackLeftMotor = hardwareMap.get(DcMotorEx.class, "bl");
        dtBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.dtFrontLeftMotor = hardwareMap.get(DcMotorEx.class, "fl");
        dtFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.dtBackRightMotor = hardwareMap.get(DcMotorEx.class, "br");
        dtBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtBackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.dtFrontRightMotor = hardwareMap.get(DcMotorEx.class, "fr");
        dtFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtFrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        left_lift = hardwareMap.get(DcMotorEx.class, "left_lift");
        right_lift = hardwareMap.get(DcMotorEx.class, "right_lift");
        left_lift.setDirection(DcMotorSimple.Direction.REVERSE);
        right_lift.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLiftEncoder = new WEncoder(new MotorEx(hardwareMap, "left_lift").encoder);
        rightLiftEncoder = new WEncoder(new MotorEx(hardwareMap, "right_lift").encoder);

        leftAnalogInput = hardwareMap.get(AnalogInput.class, "left");
        rightAnalogInput = hardwareMap.get(AnalogInput.class, "right");

        // TODO: tune arm1, motion profile, feedforward, and error tolerance
        this.leftLiftActuator = new WActuatorGroup(right_lift, rightLiftEncoder)
                .setPIDController(new PIDController(4, 0, 0.05))
                .setMotionProfile(0, new ProfileConstraints(6, 6, 5))
                .setFeedforward(WActuatorGroup.FeedforwardMode.ANGLE_BASED, 0.07, 0.2)
                .setErrorTolerance(0.03);

        this.rightLiftActuator = new WActuatorGroup(right_lift, rightLiftEncoder)
                .setPIDController(new PIDController(4, 0, 0.05))
                .setMotionProfile(0, new ProfileConstraints(6, 6, 5))
                .setFeedforward(WActuatorGroup.FeedforwardMode.ANGLE_BASED, 0.07, 0.2)
                .setErrorTolerance(0.03);

        this.leftArmEncoder = new AbsoluteAnalogEncoder(leftAnalogInput);
        leftArmEncoder.zero(2.086);
        leftArmEncoder.setInverted(true);
        leftArmEncoder.setWraparound(true);

        this.rightArmEncoder = new AbsoluteAnalogEncoder(rightAnalogInput);
        rightArmEncoder.zero(2.086);
        rightArmEncoder.setInverted(true);
        rightArmEncoder.setWraparound(true);
        
        this.leftArmActuator = new WActuatorGroup(leftArm, leftArmEncoder)
                .setPIDController(new PIDController(4, 0, 0.05))
                .setMotionProfile(0, new ProfileConstraints(6, 6, 5))
                .setFeedforward(WActuatorGroup.FeedforwardMode.ANGLE_BASED, 0.07, 0.2)
                .setErrorTolerance(0.03);

        this.rightArmActuator = new WActuatorGroup(rightArm, rightArmEncoder)
                .setPIDController(new PIDController(4, 0, 0.05))
                .setMotionProfile(0, new ProfileConstraints(6, 6, 5))
                .setFeedforward(WActuatorGroup.FeedforwardMode.ANGLE_BASED, 0.07, 0.2)
                .setErrorTolerance(0.03);

//        this.podLeft = new WEncoder(new MotorEx(hardwareMap, "dtFrontRightMotor").encoder);
//        this.podFront = new WEncoder(new MotorEx(hardwareMap, "dtBackRightMotor").encoder);
//        this.podRight = new WEncoder(new MotorEx(hardwareMap, "dtBackLeftMotor").encoder);

    }

    public void read() {
//        imuAngle = imu.getAngularOrientation().firstAngle;
        for (WSubsystem subsystem : subsystems) {
            subsystem.read();
        }
    }

    public void write() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.write();
        }
    }

    public void periodic() {
        if (voltageTimer.seconds() > 5) {
            voltageTimer.reset();
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        }

        for (WSubsystem subsystem : subsystems) {
            subsystem.periodic();
        }
    }

    public void reset() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.reset();
        }
    }

    public void clearBulkCache() {
        modules.get(0).clearBulkCache();
        modules.get(1).clearBulkCache();
    }

    public void addSubsystem(WSubsystem... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }

    @Nonnegative
    public double getAngle() {
        return imuAngle;
    }

    public double getVoltage() {
        return voltage;
    }

    public void log(String data) {
        telemetry.addLine(data);
    }

    public void log(String data, Object input) {
        telemetry.addData(data, input.toString());
    }
}