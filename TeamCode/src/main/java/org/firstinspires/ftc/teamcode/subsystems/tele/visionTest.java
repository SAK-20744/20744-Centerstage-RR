//package org.firstinspires.ftc.teamcode.subsystems.tele;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.subsystems.vision.callMainPipeline;
//
//@TeleOp
//public class visionTest extends LinearOpMode {
//    callMainPipeline cp;
//    @Override
//    public void runOpMode() {
//        telemetry.addLine("Press X for the Blue Element\nPress B for the Red Element");
//        telemetry.update();
//        while (!gamepad1.x && !gamepad1.b);
//        if (gamepad1.x) {cp = new callMainPipeline(hardwareMap, "blue");}
//        else {cp = new callMainPipeline(hardwareMap, "red");}
//
//
//        cp.StartPipeline();
//        while (!opModeIsActive()) {
//            telemetry.addData("Element Position", cp.getElementPos());
//            telemetry.update();
//            if (isStopRequested()) {
//                cp.StopPipeline();
//                break;
//            }
//        }
//        cp.StopPipeline();
//        waitForStart();
//
//    }
//}