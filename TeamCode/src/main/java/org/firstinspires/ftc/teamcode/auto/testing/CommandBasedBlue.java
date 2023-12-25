package org.firstinspires.ftc.teamcode.auto.testing;

        import android.util.Size;

        import com.acmerobotics.dashboard.FtcDashboard;
        import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
        import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.acmerobotics.roadrunner.trajectory.Trajectory;
        import com.arcrobotics.ftclib.command.CommandOpMode;
        import com.arcrobotics.ftclib.command.CommandScheduler;
        import com.arcrobotics.ftclib.command.SequentialCommandGroup;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;

        import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.firstinspires.ftc.teamcode.subsystems.PropPipeline;
        import org.firstinspires.ftc.teamcode.util.common.commandbase.driveFollowTrajectory;
        import org.firstinspires.ftc.teamcode.util.common.hardware.Globals;
        import org.firstinspires.ftc.teamcode.util.common.hardware.RobotHardware;
        import org.firstinspires.ftc.teamcode.util.common.vision.Side;
        import org.firstinspires.ftc.teamcode.subsystems.drive.SampleMecanumDrive;
        import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Disabled
@Autonomous(name = "Command Based Blue")
public class CommandBasedBlue extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private SampleMecanumDrive drive;
    private PropPipeline propPipeline;
    private VisionPortal portal;


    private double loopTime = 0.0;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = true;
        Globals.IS_USING_IMU = false;
        Globals.USING_DASHBOARD = true;
        Globals.COLOR = Side.BLUE;

        robot.init(hardwareMap, telemetry);
        robot.enabled = true;
        drive = new SampleMecanumDrive(hardwareMap);

        propPipeline = new PropPipeline();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1920, 1080))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(propPipeline)
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.read();
        while (!isStarted()) {
            telemetry.addLine("auto in init");
            telemetry.addData("POS", propPipeline.getLocation());
            telemetry.update();
        }

//        Side side = propPipeline.getLocation();
        portal.close();

        Pose2d yellowScorePos = new Pose2d();
        Pose2d purpleScorePos = new Pose2d();
        Pose2d parkPos = new Pose2d();

        Pose2d start = drive.getPoseEstimate();
        Pose2d MiddleTile = new Pose2d(12, -2, Math.toRadians(0));
        Pose2d spike1 = new Pose2d(28, 11, Math.toRadians(0));
        Pose2d spike2 = new Pose2d(32, -4, Math.toRadians(0));
        Pose2d spike3 = new Pose2d(30.5, -6.75, Math.toRadians(-90));
        Pose2d boardLeft = new Pose2d(16.5, 25, Math.toRadians(90));
        Pose2d boardMiddle = new Pose2d(25, 25, Math.toRadians(90));
        Pose2d boardRight = new Pose2d(33.5, 25, Math.toRadians(90));
        Pose2d park = new Pose2d(0,32,Math.toRadians(90));

        // 0.3, 300

//        switch (side) {
//            case LEFT:
//                yellowScorePos = boardLeft;
//                purpleScorePos = spike1;
//                parkPos = park;
//                break;
//            case CENTER:
//                yellowScorePos = boardMiddle;
//                purpleScorePos = spike2;
//                parkPos = park;
//                break;
//            case RIGHT:
//                yellowScorePos = boardRight;
//                purpleScorePos = spike3;
//                parkPos = park;
//                break;
//            default:
//                break;
//
//        }

        Trajectory toYellow = drive.trajectoryBuilder(start)
                .lineToLinearHeading(yellowScorePos)
                .build();
        Trajectory toPurple = drive.trajectoryBuilder(yellowScorePos)
                .lineToLinearHeading(purpleScorePos)
                .build();
        Trajectory toPark = drive.trajectoryBuilder(purpleScorePos)
                .lineToLinearHeading(parkPos)
                .build();


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // go to yellow pixel scoring position
                        new driveFollowTrajectory(drive, toYellow),

                        // go to purple puxel scoring position
                        new driveFollowTrajectory(drive, toPurple)
                )
        );
    }

    @Override
    public void run() {
        robot.read();
        super.run();
        robot.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

        robot.write();
        robot.clearBulkCache();
    }
}