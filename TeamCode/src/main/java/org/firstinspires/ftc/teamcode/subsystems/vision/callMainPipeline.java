package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.readCalibFile;
import org.firstinspires.ftc.teamcode.subsystems.vision.OldPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class callMainPipeline {
    OldPipeline firstPipeline;
    HardwareMap hardwareMap;
    OpenCvCamera camera;
    String allianceTemp;
    public callMainPipeline(HardwareMap hm, String alliance) {hardwareMap = hm;allianceTemp = alliance;}

    public void StartPipeline(){
        //Camera + Pipline Code
        new readCalibFile();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName,cameraMonitorViewId);
        camera.openCameraDevice();
        //camera.showFpsMeterOnViewport(false);
        camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
        firstPipeline = new OldPipeline(1280, 720);
        //firstPipeline = new readPipeline();
        camera.setPipeline(firstPipeline);
        setAlliance(allianceTemp);
    }

    public void StopPipeline(){
        camera.stopStreaming();
    }

    public int getElementPos() {
        return firstPipeline.getElementPosition();
    }
    public void setAlliance(String alliance) {firstPipeline.setAlliance(alliance);}
}