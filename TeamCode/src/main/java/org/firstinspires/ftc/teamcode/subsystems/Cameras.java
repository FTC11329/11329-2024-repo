package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utility.AprilTagToRoadRunner;
import org.firstinspires.ftc.teamcode.utility.BarcodePosition;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.processors.BarcodeProcessor;
import org.firstinspires.ftc.teamcode.vision.processors.DashboardCameraStreamProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Optional;

public class Cameras {
    boolean wasBack = true;

    public VisionPortal switchingCamera;

    WebcamName backWebcam;
    WebcamName frontWebcam;

    AprilTagProcessor aprilTagProcessor;
    public BarcodeProcessor barcodeProcessor = new BarcodeProcessor();
    DashboardCameraStreamProcessor dashboardCameraStreamProcessor = new DashboardCameraStreamProcessor();

    public Cameras(HardwareMap hardwareMap) {
        backWebcam  = hardwareMap.get(WebcamName.class, Constants.Vision.backWebcamName);
        frontWebcam = hardwareMap.get(WebcamName.class, Constants.Vision.frontWebcamName);

        CameraName switchableCameraName = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(backWebcam, frontWebcam);

        aprilTagProcessor = AprilTagDetectionPipeline.createAprilTagProcessor();

        switchingCamera = new VisionPortal
                .Builder()
                .setCamera(switchableCameraName)
                .setCameraResolution(new Size(1280, 720))
//                .addProcessor(barcodeProcessor)
                .addProcessor(aprilTagProcessor)
                .enableLiveView(false)
                .build();

        FtcDashboard.getInstance().startCameraStream(dashboardCameraStreamProcessor, 30);

        switchingCamera.setActiveCamera(backWebcam);
    }


    public ArrayList<AprilTagDetection> getAprilTagRecognitions() {
        return aprilTagProcessor.getDetections();
    }

    public void resumeStreaming() {
        switchingCamera.resumeStreaming();
    }

    public void stopStreaming() {
        switchingCamera.stopStreaming();
    }

    public void kill() {
        switchingCamera.close();
    }

    public void setCameraSide(boolean setBack) {
        if (switchingCamera.getCameraState() == VisionPortal.CameraState.STREAMING) {
            if (setBack && !wasBack) {

                switchingCamera.setActiveCamera(backWebcam);
            } else if (!setBack && wasBack) {
                switchingCamera.setActiveCamera(frontWebcam);
            }
            wasBack = setBack;
        }
    }

    public void setCameraSideThreaded(boolean setBack) {
        new CameraSwitchingThread(this, setBack).start();
    }

    public Optional<Pose2d> getRunnerPoseEstimate(int id, boolean isBack) {
        isBack = switchingCamera.getActiveCamera().equals(backWebcam);

        Optional<AprilTagDetection> desiredTag = Optional.empty();
        if (id == 0) {
            //for cycling through all of them
            for (int i = 1; i < 13; i++) {
                desiredTag = AprilTagDetectionPipeline.getDesiredTag(aprilTagProcessor.getDetections(), i);
                if (desiredTag.isPresent()) {
                    break;
                }
            }
        } else {
            desiredTag = AprilTagDetectionPipeline.getDesiredTag(aprilTagProcessor.getDetections(), id);
        }

        if (!desiredTag.isPresent()) return Optional.empty();

        Pose2d pose = AprilTagToRoadRunner.tagToRunner(desiredTag.get(), isBack);
        return Optional.of(pose);
    }

    public Optional<BarcodePosition> getBarcodePosition() {
        return barcodeProcessor.getLastKnownPosition();
    }
}


class CameraSwitchingThread extends Thread {
    Cameras cameras;
    boolean sideToSwitchTo;

    public CameraSwitchingThread(Cameras cameras, boolean sideToSwitchTo) {
        super();
        this.cameras = cameras;
        this.sideToSwitchTo = sideToSwitchTo;
    }

    @Override
    public void run() {
        cameras.setCameraSide(sideToSwitchTo);
    }
}