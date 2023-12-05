package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection.Location.CENTER;
import static org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection.Location.LEFT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm1;
import org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection;
import org.firstinspires.ftc.teamcode.subsystems.ServoArm;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

//@Disabled
@Autonomous
public class BlueFull extends LinearOpMode {

    private CenterStageDetection detector;
    private SampleMecanumDrive drive;
    private Arm1 arm1;
    private ServoArm arm2;

    private Servo wrist;
    private DcMotor left_lift;
    private DcMotor right_lift;
    private CRServo leftArm;
    private CRServo rightArm;
    private CRServo intake;
    private Servo door;

    private boolean ButtonXBlock = false;
    private double wristservoposition = 0.63;

    private boolean isLeft = false;
    private boolean isMiddle = false;
    private boolean isRight = false;

    private CenterStageDetection.ColorDetected colorLeft;
    private CenterStageDetection.ColorDetected colorMiddle;

    private OpenCvCamera camera;

    private CenterStageDetection.Location location;

    @Override
    public void runOpMode() throws InterruptedException{

        arm2 = new ServoArm(hardwareMap);
        arm1 = new Arm1(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);

        detector = new CenterStageDetection();

        door = hardwareMap.get(Servo.class, "door");
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");

        rightArm = hardwareMap.get(CRServo.class, "rightArm");
        leftArm = hardwareMap.get(CRServo.class, "leftArm");

        right_lift = hardwareMap.get(DcMotor.class, "right_lift");
        left_lift = hardwareMap.get(DcMotor.class, "left_lift");

        int width = 320;
        int height = 240;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.openCameraDevice();
        camera.setPipeline(detector);
        camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);

        doInit();

        Pose2d startPose = new Pose2d(0,0,Math.toRadians(90));

        Pose2d leftSpike = new Pose2d(-6,36, Math.toRadians(90));
        Pose2d centerSpike = new Pose2d(8,36, Math.toRadians(90));
        Pose2d rightSpike = new Pose2d(12,36, Math.toRadians(45));

        Pose2d leftBackdrop = new Pose2d(-22, -19, Math.toRadians(180));
        Pose2d centerBackdrop = new Pose2d(-22, -25, Math.toRadians(180));
        Pose2d rightBackdrop = new Pose2d(-22, -35, Math.toRadians(180));

        Pose2d outOfWay = new Pose2d(-22,0, Math.toRadians(180));
        Pose2d parkPos = new Pose2d(-28, 0, Math.toRadians(180));

        TrajectorySequence toLeft = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(leftSpike)
                .addDisplacementMarker(() -> {
                    moveArmsLeft();
                    dropPurple();
                })
                .lineToLinearHeading(leftBackdrop)
                .addDisplacementMarker(() ->{
                    dropYellow();
                })
                .lineToLinearHeading(outOfWay)
                .lineToLinearHeading(parkPos)
                .build();

        TrajectorySequence toCenter = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(centerSpike)
                .addDisplacementMarker(() -> {
                   moveArmsCenter();
                   dropPurple();
                })
                .lineToLinearHeading(centerBackdrop)
                .addDisplacementMarker(() ->{
                    dropYellow();
                })
                .lineToLinearHeading(outOfWay)
                .lineToLinearHeading(parkPos)
                .build();

        TrajectorySequence toRight = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(rightSpike)
                .addDisplacementMarker(() -> {
                   moveArmsRight();
                   dropPurple();
                })
                .lineToLinearHeading(rightBackdrop)
                .addDisplacementMarker(() ->{
                    dropYellow();
                })
                .lineToLinearHeading(outOfWay)
                .lineToLinearHeading(parkPos)
                .build();

        Trajectory right1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(rightSpike)
                .build();


        waitForStart();
        if (!isStopRequested()) {

            detect();

            if (location == LEFT) {
                drive.followTrajectorySequence(toLeft);
            }

            else if (location == CENTER) {
                drive.followTrajectorySequence(toCenter);
            }

            else {
                drive.followTrajectorySequence(toRight);
            }

        }

    }

    public void detect() {
        leftArm.setPower(0.45);
        rightArm.setPower(-0.45);
        sleep(300);
        leftArm.setPower(0);
        rightArm.setPower(0);
        wrist.setPosition(0.63);
        door.setPosition(0);
        arm2.runToProfile(0);
        sleep(2000);
        colorLeft = detector.getColorLeft();
        colorMiddle = detector.getColorMiddle();
        location = detector.getLocation();
        telemetry.addData("Detecting Left: ", colorLeft);
        telemetry.addData("Detecting Center: ", colorMiddle);
        telemetry.addData("Location", location);
        telemetry.update();
        camera.stopStreaming();
    }

    public void doInit() {

        wrist.setPosition(0.63);
        door.setPosition(0.75);

        while (!(opModeIsActive() || isStopRequested())) {
            left_lift.setPower(-gamepad2.right_stick_y);
            right_lift.setPower(-gamepad2.right_stick_y);
            leftArm.setPower(-gamepad2.left_stick_y);
            rightArm.setPower(gamepad2.left_stick_y);
            telemetry.addData("Left Lift Encoder", left_lift.getCurrentPosition());
            telemetry.addData("Right Lift Encoder", right_lift.getCurrentPosition());
            telemetry.addData("Servo Arm", arm2.getLocation());
            telemetry.update();
            if (gamepad2.a) {
                left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                left_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }

    public void dropPurple() {
        door.setPosition(0.75);
        sleep(500);
        door.setPosition(0);
        sleep(500);
        intake.setPower(-0.5);
        sleep(500);
        intake.setPower(0);
        sleep(500); 
    }

    public void moveArmsLeft() {
        wrist.setPosition(0.21);
        arm1.ArmToPos(-2000,1);
        arm2.runToProfile(-7.5);
        while( (arm2.isBusy()) && !isStopRequested()) {
            arm2.updateServoArm();
            telemetry.addData("Position", "Left");
            telemetry.addData("Arm2" , arm2.getLocation());
            telemetry.addData("Arm2 State" , arm2.isBusy());
            telemetry.update();
        }
    }

    public void moveArmsCenter() {
        wrist.setPosition(0.21);
        arm1.ArmToPos(-2000,1);
        arm2.runToProfile(-7.5);
        while( (arm2.isBusy()) && !isStopRequested()) {
            arm2.updateServoArm();
            telemetry.addData("Position", "Center");
            telemetry.addData("Arm2" , arm2.getLocation());
            telemetry.addData("Arm2 State" , arm2.isBusy());
            telemetry.update();
        }
    }

    public void moveArmsRight() {
        wrist.setPosition(0.21);
        arm1.ArmToPos(-2000,1);
        arm2.runToProfile(-7.5);
        while( (arm2.isBusy()) && !isStopRequested()) {
            arm2.updateServoArm();
            telemetry.addData("Position", "Right");
            telemetry.addData("Arm2" , arm2.getLocation());
            telemetry.addData("Arm2 State" , arm2.isBusy());
            telemetry.update();
        }
    }

    public void dropYellow() {
        door.setPosition(0.75);
        sleep(500);
        door.setPosition(0);
    }
}