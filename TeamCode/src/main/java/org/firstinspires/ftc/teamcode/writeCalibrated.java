//package org.firstinspires.ftc.teamcode;
//
//import android.view.ViewDebug;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ReadWriteFile;
//
//import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
//import org.firstinspires.ftc.teamcode.subsystems.vision.OldPipeline;
//
//import java.io.BufferedReader;
//import java.io.File;
//import java.io.FileNotFoundException;
//import java.io.InputStreamReader;
//import java.io.Reader;
//import java.util.Scanner;
//
//@TeleOp (name = "write calibrated values")
//public class writeCalibrated extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        File cameraDetection = AppUtil.getInstance().getSettingsFile("visionCalibration.txt");
//        String RVals = (
////Red
//                OldPipeline.RH_MIN + "\n" +
//                        OldPipeline.RS_MIN + "\n" +
//                        OldPipeline.RV_MIN + "\n" +
//                        OldPipeline.RH_MAX + "\n" +
//                        OldPipeline.RS_MAX + "\n" +
//                        OldPipeline.RV_MAX + "\n"
//        );
//        String BVals = (
////Blue
//                OldPipeline.BH_MIN + "\n" +
//                        OldPipeline.BS_MIN + "\n" +
//                        OldPipeline.BV_MIN + "\n" +
//                        OldPipeline.BH_MAX + "\n" +
//                        OldPipeline.BS_MAX + "\n" +
//                        OldPipeline.BV_MAX + "\n"
//        );
//        String calibValues = (
//                RVals + "" + BVals
//        );
//        ReadWriteFile.writeFile(cameraDetection, calibValues);
//        String read = ReadWriteFile.readFile(cameraDetection);
//        /*try {
//            Scanner scan = new Scanner(readWriteFile1);
//            line=scan.nextLine();
//
//        } catch (FileNotFoundException e) {
//            e.printStackTrace();
//        }*/
//
//        telemetry.addData("Values: ", ("Red\n" +
//                RVals + ",\n" +
//                "Blue\n" +
//                BVals + ",\n"));
//        telemetry.update();
//        waitForStart();
//        while (opModeIsActive()) {
//            if (isStopRequested())
//                break;
//            telemetry.addData("Values: ", ("Red\n" +
//                    RVals + ",\n" +
//                    "Blue\n" +
//                    BVals + ",\n"));
//            telemetry.update();
//        }
//    }
//}