package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Cameras;
import org.firstinspires.ftc.teamcode.utility.BarcodePosition;

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
        if (cameras.isZoomCorrectionLegal()) {
            cameras.correctZoom();
        }

        Optional<BarcodePosition> positionOptional = cameras.getBarcodePosition();


        if (positionOptional.isPresent()) {
            telemetry.addData("Known Barcode Position ", positionOptional.get());
        } else {
            telemetry.addLine("No Barcode position found");
        }
        telemetry.update();
    }
}
