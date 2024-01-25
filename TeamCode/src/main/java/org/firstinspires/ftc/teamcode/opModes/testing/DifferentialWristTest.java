package org.firstinspires.ftc.teamcode.opModes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class DifferentialWristTest extends LinearOpMode {

    // Define your CRServo objects
    private CRServo leftServo;
    private CRServo rightServo;

    // Define analog encoder objects
    private AnalogInput leftEncoder;
    private AnalogInput rightEncoder;

    // Desired pitch and roll angles
    public static double desiredPitch = 0.0;  // Set your desired pitch angle
    public static double desiredRoll = 0.0;   // Set your desired roll angle

    // Other necessary variables
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize your CRServos and encoders (make sure to map them in your hardware configuration)
        leftServo = hardwareMap.crservo.get("leftServo");
        rightServo = hardwareMap.crservo.get("rightServo");
        leftEncoder = hardwareMap.analogInput.get("leftEncoder");
        rightEncoder = hardwareMap.analogInput.get("rightEncoder");

        // Wait for the start button to be pressed
        waitForStart();
        runtime.reset();

        // Your code for controlling servos and setting differential wrist angles
        while (opModeIsActive()) {

            double leftAngle = leftEncoder.getVoltage() / leftEncoder.getMaxVoltage() * 360;
            double rightAngle = rightEncoder.getVoltage() / rightEncoder.getMaxVoltage() * 360;

            double pitchPower = angleToPower(desiredPitch);
            double rollPower = angleToPower(desiredRoll);

            // Applying constraints
            pitchPower = constrainServoPower(pitchPower);
            rollPower = constrainServoPower(rollPower);

            // Setting CRServo powers for both servos
            leftServo.setPower(pitchPower + rollPower);
            rightServo.setPower(pitchPower - rollPower);

            // Display angles on telemetry (optional)
            telemetry.addData("Left Angle", leftAngle);
            telemetry.addData("Right Angle", rightAngle);
            telemetry.update();

        }
    }

    // Function to convert angle to CRServo power
    private double angleToPower(double angle) {
        // Assuming a linear relationship between angle and power
        return angle / 180.0;  // Convert to CRServo power range (-1.0 to 1.0)
    }

    // Function to constrain CRServo power within the valid range (-1.0 to 1.0)
    private double constrainServoPower(double power) {
        return Math.max(-1.0, Math.min(power, 1.0));
    }
}