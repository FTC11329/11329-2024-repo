package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.teamcode.Constants;
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
    VisionPortal frontCamera;

    AprilTagProcessor aprilTag;
    BarcodeProcessor barcodeProcessor = new BarcodeProcessor();
    DashboardCameraStreamProcessor dashboardCameraStreamProcessor = new DashboardCameraStreamProcessor();

    public Cameras(HardwareMap hardwareMap) {
        aprilTag = AprilTagDetectionPipeline.createAprilTagProcessor();

        frontCamera = new VisionPortal
                .Builder()
                .setCamera(hardwareMap.get(WebcamName.class, Constants.Vision.frontWebcamName))
                .addProcessor(barcodeProcessor)
                .addProcessor(aprilTag)
                .addProcessor(dashboardCameraStreamProcessor)
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

    public Optional<Pose2d> getPoseEstimate() {
        Optional<AprilTagDetection> desiredTag = AprilTagDetectionPipeline.getDesiredTag(aprilTag.getDetections(), 1);

        if (!desiredTag.isPresent()) return Optional.empty();

        Pose2d pose = new Pose2d(desiredTag.get().ftcPose.x, desiredTag.get().ftcPose.y, desiredTag.get().ftcPose.yaw);

        return Optional.of(pose);
    }

    public Optional<BarcodePosition> getBarcodePosition() {
        return barcodeProcessor.getLastKnownPosition();
    }
}
