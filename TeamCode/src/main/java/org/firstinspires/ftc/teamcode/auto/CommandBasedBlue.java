package org.firstinspires.ftc.teamcode.auto;

        import android.util.Size;

        import com.acmerobotics.dashboard.FtcDashboard;
        import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
        import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.arcrobotics.ftclib.command.CommandOpMode;
        import com.arcrobotics.ftclib.command.CommandScheduler;
        import com.arcrobotics.ftclib.command.SequentialCommandGroup;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.firstinspires.ftc.teamcode.common.hardware.Globals;
        import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
        import org.firstinspires.ftc.teamcode.common.vision.Side;
        import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
        import org.firstinspires.ftc.teamcode.common.vision.PropPipeline;
        import org.firstinspires.ftc.vision.VisionPortal;

@Config
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

        Side side = propPipeline.getLocation();
        portal.close();

        Pose2d yellowScorePos = new Pose2d();
        Pose2d purpleScorePos = new Pose2d();
        Pose2d parkPos = new Pose2d();

        // 0.3, 300

        switch (side) {
            case LEFT:
                yellowScorePos = new Pose2d(21.5, -23.25, 1.52);
                purpleScorePos = new Pose2d(26, -25, 1.52);
                parkPos = new Pose2d(6, -31, 3 * Math.PI / 2);
                break;
            case CENTER:
                yellowScorePos = new Pose2d(27.75, -23.25, 1.52);
                purpleScorePos = new Pose2d(36, -18, 1.52);
                parkPos = new Pose2d(6, -31, 3 * Math.PI / 2);
                break;
            case RIGHT:
                yellowScorePos = new Pose2d(34.25, -23.25, 1.52);
                purpleScorePos = new Pose2d(24, -4, 1.52);
                parkPos = new Pose2d(6, -31, 3 * Math.PI / 2);
                break;
            default:
                break;

        }


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
//                        // go to yellow pixel scoring pos
//                        new PositionCommand((Drivetrain) drivetrain, localizer, yellowScorePos)
//                                .alongWith(new YellowPixelExtendCommand(robot, extension, intake)),
//
//                        // score yellow pixel
//                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.ClawState.INTERMEDIATE, ClawSide.RIGHT)),
//                        new WaitCommand(200),
//
//                        // retract
//                        new YellowPixelRetractCommand(robot, extension, intake, ClawSide.RIGHT),
//
//                        // go to purple pixel scoring pos
//                        new PositionCommand((Drivetrain) drivetrain, localizer, purpleScorePos)
//                                .alongWith(new PurplePixelExtendCommand(robot, extension, intake)),
//
//                        // score purple pixel
//                        new WaitCommand(500),
//                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.ClawState.OPEN, ClawSide.LEFT)),
//                        new WaitCommand(350),
//
//                        new PurplePixelRetractCommand(robot, extension, intake, ClawSide.LEFT),
//
//                        new PositionCommand((Drivetrain) drivetrain, localizer, parkPos)
//                                .alongWith(new WaitCommand(400).andThen(new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0.0475))))
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
//        telemetry.addLine(localizer.getPos().toString());
        loopTime = loop;
        telemetry.update();

        robot.write();
        robot.clearBulkCache();
    }
}