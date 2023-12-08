package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystems.vision.OldPipeline;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public class readCalibFile {

    public readCalibFile() {
        File cameraDetection = AppUtil.getInstance().getSettingsFile("visionCalibration.txt");
        try {
//Red
            Scanner scan = new Scanner(cameraDetection);
            OldPipeline.RH_MIN = Integer.parseInt(scan.nextLine());;
            OldPipeline.RS_MIN = Integer.parseInt(scan.nextLine());
            OldPipeline.RV_MIN = Integer.parseInt(scan.nextLine());
            OldPipeline.RH_MAX = Integer.parseInt(scan.nextLine());
            OldPipeline.RS_MAX = Integer.parseInt(scan.nextLine());
            OldPipeline.RV_MAX = Integer.parseInt(scan.nextLine());
//Blue
            OldPipeline.BH_MIN = Integer.parseInt(scan.nextLine());
            OldPipeline.BS_MIN = Integer.parseInt(scan.nextLine());
            OldPipeline.BV_MIN = Integer.parseInt(scan.nextLine());
            OldPipeline.BH_MAX = Integer.parseInt(scan.nextLine());
            OldPipeline.BS_MAX = Integer.parseInt(scan.nextLine());
            OldPipeline.BV_MAX = Integer.parseInt(scan.nextLine());
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        String calibValues = (
//Red
                OldPipeline.RH_MIN + ",\n" +
                        OldPipeline.RS_MIN + ",\n" +
                        OldPipeline.RV_MIN + ",\n" +
                        OldPipeline.RH_MAX + ",\n" +
                        OldPipeline.RS_MAX + ",\n" +
                        OldPipeline.RV_MAX + ",\n" +
//Blue
                        OldPipeline.BH_MIN + ",\n" +
                        OldPipeline.BS_MIN + ",\n" +
                        OldPipeline.BV_MIN + ",\n" +
                        OldPipeline.BH_MAX + ",\n" +
                        OldPipeline.BS_MAX + ",\n" +
                        OldPipeline.BV_MAX + ",\n"
        );
        readCalibrated.read = calibValues;
    }
}