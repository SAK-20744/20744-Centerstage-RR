//package org.firstinspires.ftc.teamcode.auto
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
//import com.qualcomm.robotcore.hardware.DcMotor
//import com.qualcomm.robotcore.hardware.DcMotorEx
//import com.qualcomm.robotcore.hardware.DcMotorSimple
//import com.qualcomm.robotcore.hardware.HardwareMap
//import com.qualcomm.robotcore.hardware.IMU
//import com.qualcomm.robotcore.hardware.Servo
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
//import org.firstinspires.ftc.teamcode.GAmovement
//import org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection
//import org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection.ColorDetected
//import org.firstinspires.ftc.teamcode.subsystems.intake
//import org.firstinspires.ftc.teamcode.subsystems.lift
//import org.firstinspires.ftc.teamcode.subsystems.turret
//import org.firstinspires.ftc.vision.VisionPortal
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
//import org.openftc.easyopencv.OpenCvCamera
//import org.openftc.easyopencv.OpenCvCameraFactory
//import org.openftc.easyopencv.OpenCvCameraRotation
//import org.openftc.easyopencv.OpenCvInternalCamera
//
////import org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection;
//@Autonomous
//class AutoCenterStage : LinearOpMode() {
//    var isLeft = false
//    var isMiddle = false
//    var isRight = false
//    //val tagProcessor = AprilTagProcessor.easyCreateWithDefaults()
//    var runAut = false
//    override fun runOpMode() {
//        val detector = CenterStageDetection()
//        val camera: OpenCvCamera
//        isLeft = false
//        isMiddle = false
//        isRight = false
//        val width = 320
//        val height = 240
//        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
//            "cameraMonitorViewId",
//            "id",
//            hardwareMap.appContext.packageName
//        )
//        camera = OpenCvCameraFactory.getInstance()
//            .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId)
//        camera.openCameraDevice()
//        camera.setPipeline(detector)
//        camera.startStreaming(width, height, OpenCvCameraRotation.SIDEWAYS_LEFT)
//        var colorLeft = detector.colorLeft
//        var colorMiddle = detector.colorMiddle
//        telemetry.addData("Detecting Left: ", colorLeft)
//        telemetry.addData("Detecting Center: ", colorMiddle)
//        telemetry.update()
//
//        val imu = hardwareMap.get(IMU::class.java, "imu")
//        var x = IMU.Parameters(
//            RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
//                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
//            )
//        )
//        imu.initialize(x)
//
//        //ENTER MOTOR NAMES AND STUFF IN HERE:
//         val m = Move(
//            hardwareMap.get(DcMotorEx::class.java, "leftFront"),
//            hardwareMap.get(DcMotorEx::class.java, "rightFront"),
//            hardwareMap.get(DcMotorEx::class.java, "leftRear"),
//            hardwareMap.get(DcMotorEx::class.java, "rightRear"),
//            hardwareMap,
//            hardwareMap.get(DcMotorEx::class.java, "leftRear"),
//            hardwareMap.get(DcMotorEx::class.java, "rightRear"),
//            imu,
//            this
//        );
//        waitForStart()
//
////        val camera1 = hardwareMap.get(WebcamName::class.java, "Webcam 1")
////        val portal = VisionPortal.easyCreateWithDefaults(camera1, tagProcessor)
//        var i = 0
//        while (!isStopRequested) {
//            colorLeft = detector.colorLeft
//            colorMiddle = detector.colorMiddle
//            if (colorLeft == ColorDetected.CYAN || colorLeft == ColorDetected.MAGENTA) isLeft =
//                true else if (colorMiddle == ColorDetected.CYAN || colorMiddle == ColorDetected.MAGENTA) isMiddle =
//                true else isRight = true
//            telemetry.addData("Detecting Left: ", colorLeft)
//            telemetry.addData("Detecting Center: ", colorMiddle)
//
//
//            if (i==500) {
//                camera.stopStreaming()
//                runAut = true
//            }
//
//            m.update()
//            telemetry.addData("Lift Pos:", m.Lift.whereAmI())
//            telemetry.addData("Turret Pos:", m.Turret.whereAmI())
//            telemetry.update()
//            i++
//        }
//
////        if (isLeft) {
////            // Movements for left spot
////            telemetry.addData("Position", "Left");
////            telemetry.update();
////            sleep(20000);
////        }
////        else if (isMiddle) {
////            // Movements for center spot
////            telemetry.addData("Position", "Right");
////            telemetry.update();
////            sleep(20000);
////        }
////        else {
////            // Movements for right spot
////            telemetry.addData("Position", "Middle");
////            telemetry.update();
////            sleep(20000);
////        }
//    }
//}
//
//internal class Move(
//    var fl: DcMotorEx,
//    var fr: DcMotorEx,
//    var bl: DcMotorEx,
//    var br: DcMotorEx,
//    var hMap: HardwareMap,
//    StrafeOdometer: DcMotorEx,
//    DriveOdometer: DcMotorEx,
//    var Imu: IMU,
//    var a: AutoCenterStage
//) : GAmovementKotlinRewrite(
//    fl,
//    fr,
//    bl,
//    br,
//    hMap,
//    StrafeOdometer,
//    DriveOdometer,
//    Imu
//) {
//    val Intake = intake(hMap)
//    val Lift = lift(hMap, Intake)
//    val Turret = turret(hMap)
//
//    var loc = 0
//    private val claw: Servo = hMap.get<Servo>(Servo::class.java, "intake")
//    val liftmotor = hMap.get<DcMotor?>(DcMotor::class.java, "lift")
//    override fun onStart() {
//        loc = if (a.isLeft) {
//            -1
//        } else if (a.isMiddle) {
//            0
//        } else {
//            1
//        }
//        fl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//        fr.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//        bl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//        br.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//
//        fr.direction = DcMotorSimple.Direction.REVERSE
//        br.direction = DcMotorSimple.Direction.REVERSE
//
//    }
//
//    override fun autoConstructor(t: Int): Boolean {
////        loc = 1
////        val tag: Int = when (loc) {
////            1 -> 1
////            0 -> 2
////            -1 -> 3
////            else -> 0
////        }
//
////        var detections: List<AprilTagDetection?>? = a.tagProcessor.detections.ifEmpty {
////            null
////        }
////        var detect = detections?.get(0)
////        var dPose = detections?.get(0)?.ftcPose
////        var detected = false
////        if (detections != null) {
////            for (detection in detections) {
////                val pose = detection?.ftcPose
////                if (detection != null) {
////                    if (detection.id == tag) {
////                        detect = detection
////                        dPose = pose
////                        detected = true
////                    }
////                }
////            }
////        }
//        var tpi = 1800
//        when (t) {
//            0 -> {
//                claw.position = 0.0
////                synchronize()
////                if(Lift.liftLow() && a.runAut) {
////                    complete()
////                }
//            }
//            1 -> moveToTicks(18920, 0, 0.5)
//            2 -> {
//                val yaw1 = Imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)
//                fl.power = 0.5
//                bl.power = 0.5
//                br.power = -0.5
//                fr.power = -0.5
//                while (Imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES) < yaw1 + 90)
//                fl.power = 0.0
//                fr.power = 0.0
//                br.power = 0.0
//                bl.power = 0.0
//            }
//            3 -> {
////                if (detected && dPose != null) {
////            moveToTicks((dPose.range * tpi).toInt(), (dPose.bearing * tpi).toInt(), 0.5)
//            }
//
//            else -> {}
//        }
//        Lift.updatelift()
//        Turret.updateTurret()
//        return true
//    }
//}