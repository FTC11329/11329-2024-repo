package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    public VisionPortal frontCamera;
    public VisionPortal backCamera;

    AprilTagProcessor aprilTag;
    public BarcodeProcessor barcodeProcessor = new BarcodeProcessor();
    DashboardCameraStreamProcessor dashboardCameraStreamProcessor = new DashboardCameraStreamProcessor();

    public Cameras(HardwareMap hardwareMap) {
        aprilTag = AprilTagDetectionPipeline.createAprilTagProcessor();

//        frontCamera = new VisionPortal
//                .Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, Constants.Vision.frontWebcamName))
//                .setCameraResolution(new Size(1280, 720))
//                .addProcessor(barcodeProcessor)
////                .addProcessor(aprilTag)
//                .addProcessor(dashboardCameraStreamProcessor)
//                .enableLiveView(true)
//                .build();

        backCamera = new VisionPortal
                .Builder()
                .setCamera(hardwareMap.get(WebcamName.class, Constants.Vision.backWebcamName))
                .setCameraResolution(new Size(1280, 720))
//                .addProcessor(barcodeProcessor)
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(dashboardCameraStreamProcessor, 30);
    }

    public boolean isZoomCorrectionLegal() {
        return frontCamera.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public void correctZoom() {
        PtzControl frontWebcamControl = frontCamera.getCameraControl(PtzControl.class);

        frontWebcamControl.setZoom(frontWebcamControl.getMinZoom());
    }

    public ArrayList<AprilTagDetection> getAprilTagRecognitions() {
        return aprilTag.getDetections();
    }

    public Optional<Pose2d> getRunnerPoseEstimate(int id) {
        Optional<AprilTagDetection> desiredTag = Optional.empty();
        if (id == 0) {
            //for cycling through all of them
            for (int i = 1; i < 10; i++) {
                desiredTag = AprilTagDetectionPipeline.getDesiredTag(aprilTag.getDetections(), i);
                if (desiredTag.isPresent()) {
                    break;
                }
            }
        } else {
            desiredTag = AprilTagDetectionPipeline.getDesiredTag(aprilTag.getDetections(), id);
        }

        if (!desiredTag.isPresent()) return Optional.empty();

        Pose2d pose = AprilTagToRoadRunner.tagToRunner(desiredTag.get());
        pose = pose.minus(new Pose2d(0,0,Math.toRadians(180)));

        return Optional.of(pose);

    }

    public Optional<BarcodePosition> getBarcodePosition() {
        return barcodeProcessor.getLastKnownPosition();
    }
}
