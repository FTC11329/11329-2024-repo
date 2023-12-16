package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Cameras;

@TeleOp(name = "Vision Test", group = "test")
public class VisionTest extends OpMode {
    Cameras cameras;

    @Override
    public void init() {
        cameras = new Cameras(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Barcode Position", cameras.getBarcodePosition());
        // Pose2d finalUpdatePose = new Pose2d(0, 0, 0);
        // drivetrain.setPoseEstimate(finalUpdatePose)
    }
}