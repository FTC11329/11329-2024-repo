package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utility.BarcodePosition;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.processors.DashboardCameraStreamProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Optional;

public class Cameras {
    WebcamName backWebcam;
    WebcamName frontWebcam;
    AprilTagProcessor frontAprilTagProcessor;
    AprilTagProcessor backAprilTagProcessor;
    VisionPortal frontWebcamPortal;
    VisionPortal backWebcamPortal;


    DashboardCameraStreamProcessor dashboardCameraStreamProcessor = new DashboardCameraStreamProcessor();

    public Cameras(HardwareMap hardwareMap) {
        backWebcam = hardwareMap.get(WebcamName.class, Constants.Vision.backWebcamName);
        frontWebcam = hardwareMap.get(WebcamName.class, Constants.Vision.frontWebcamName);

        frontAprilTagProcessor = AprilTagDetectionPipeline.createAprilTagProcessor();

        frontWebcamPortal = new VisionPortal.Builder().setCamera(frontWebcam).setCameraResolution(new Size(1280, 720)).addProcessor(frontAprilTagProcessor).enableLiveView(false).build();

        backAprilTagProcessor = AprilTagDetectionPipeline.createAprilTagProcessor();

        backWebcamPortal = new VisionPortal.Builder().setCamera(backWebcam).setCameraResolution(new Size(1280, 720)).addProcessor(backAprilTagProcessor).enableLiveView(false).build();

        FtcDashboard.getInstance().startCameraStream(dashboardCameraStreamProcessor, 30);
    }

    public ArrayList<AprilTagDetection> getAprilTagRecognitions() {
        ArrayList<AprilTagDetection> list = frontAprilTagProcessor.getDetections();
        list.addAll(backAprilTagProcessor.getDetections());
        return list;
    }


    public void kill() {
        frontWebcam.close();
    }

    public void setCameraSide(boolean setBack) {
//        if (switchingCamera.getCameraState() == VisionPortal.CameraState.STREAMING) {
//            if (setBack) {
//                switchingCamera.setActiveCamera(backWebcam);
//            } else {
//                switchingCamera.setActiveCamera(frontWebcam);
//            }
//        }
    }

    public Optional<Pose2d> getRunnerPoseEstimate(int id, boolean isBack) {
//        isBack = switchingCamera.getActiveCamera().equals(backWebcam);
//
//        Optional<AprilTagDetection> desiredTag = Optional.empty();
//        if (id == 0) {
//            //for cycling through all of them
//            for (int i = 1; i < 10; i++) {
//                desiredTag = AprilTagDetectionPipeline.getDesiredTag(getAprilTagRecognitions(), i);
//                if (desiredTag.isPresent()) {
//                    break;
//                }
//            }
//        } else {
//            desiredTag = AprilTagDetectionPipeline.getDesiredTag(getAprilTagRecognitions(), id);
//        }
//
//        if (!desiredTag.isPresent()) return Optional.empty();
//
//        Pose2d pose = AprilTagToRoadRunner.tagToRunner(desiredTag.get(), isBack);
//        return Optional.of(pose);
        return Optional.empty();
    }

    public Optional<BarcodePosition> getBarcodePosition() {
//        return barcodeProcessor.getLastKnownPosition();
        return Optional.empty();
    }
}
