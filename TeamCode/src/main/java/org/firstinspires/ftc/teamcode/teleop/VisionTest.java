package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.teamcode.subsystems.Cameras;
import org.firstinspires.ftc.teamcode.utility.BarcodePosition;
import org.firstinspires.ftc.teamcode.utility.ControlUtils;

import java.util.Optional;

@TeleOp(name = "Vision Test", group = "test")
public class VisionTest extends OpMode {
    Cameras cameras;

    @Override
    public void init() {
        cameras = new Cameras(hardwareMap);
    }

    @Override
    public void loop() {
        PtzControl ptzControl = cameras.frontCamera.getCameraControl(PtzControl.class);

        if (cameras.isZoomCorrectionLegal()) {
            double zoom = ControlUtils.mapAxisToRange(gamepad1.left_stick_y, ptzControl.getMinZoom(), ptzControl.getMaxZoom());

            ptzControl.setZoom((int) Math.round(zoom));
        }

        Optional<BarcodePosition> positionOptional = cameras.getBarcodePosition();

        telemetry.addData("Camera is streaming", cameras.isZoomCorrectionLegal() ? "yes" : "no");
        telemetry.addLine("Left Stick Y controls zoom");
        telemetry.addLine();

        telemetry.addData("Current Camera Zoom", ptzControl.getZoom());
        if (positionOptional.isPresent()) {
            telemetry.addData("Known Barcode Position ", positionOptional.get());
        } else {
            telemetry.addLine("No Barcode position found");
        }

        telemetry.update();
    }
}
