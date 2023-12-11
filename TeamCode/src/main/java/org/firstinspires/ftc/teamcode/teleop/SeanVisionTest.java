package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utility.BarcodePosition;
import org.firstinspires.ftc.teamcode.vision.processors.BarcodeProcessor;
import org.firstinspires.ftc.teamcode.vision.processors.DashboardCameraStreamProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Optional;

@TeleOp(name = "Sean Vision Test", group = "test")
public class SeanVisionTest extends OpMode {
    BarcodeProcessor barcodeProcessor = new BarcodeProcessor();
    DashboardCameraStreamProcessor dashboardCameraStreamProcessor = new DashboardCameraStreamProcessor();

    VisionPortal visionPortal;

    @Override
    public void init() {
        WebcamName frontWebcam = hardwareMap.get(WebcamName.class, Constants.Vision.frontWebcamName);

        visionPortal = new VisionPortal
                .Builder()
                .setCamera(frontWebcam)
                .addProcessor(barcodeProcessor)
                .addProcessor(dashboardCameraStreamProcessor)
                .enableLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(dashboardCameraStreamProcessor, 30);
    }

    @Override
    public void loop() {
        Optional<BarcodePosition> positionOptional = barcodeProcessor.getLastKnownPosition();

        if (positionOptional.isPresent()) {
            telemetry.addData("Known Position", positionOptional.get());
        } else {
            telemetry.addLine("No position found");
        }
    }
}
