package org.firstinspires.ftc.teamcode.subsystems.vision.old.owen;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class callReadPipeline {
    readPipeline firstPipeline;
    HardwareMap hardwareMap;
    OpenCvCamera camera;

    public callReadPipeline(HardwareMap hm)
    {
        hardwareMap = hm;
    }

    public void StartPipeline(){
        //Camera + Pipline Code
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName,cameraMonitorViewId);
        camera.openCameraDevice();
        //camera.showFpsMeterOnViewport(false);
        camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
        firstPipeline = new readPipeline();
        camera.setPipeline(firstPipeline);
    }

    public void StopPipeline(){
        camera.stopStreaming();
    }

    public int getElementPos() {
        return firstPipeline.getElementPosition();
    }
    public double getH() {return firstPipeline.getH();}
    public double getS() {return firstPipeline.getS();}
    public double getV() {return firstPipeline.getV();}
}