package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Cameras;
import org.firstinspires.ftc.teamcode.utility.RobotSide;

@TeleOp(name = "Vision Test", group = "test")
public class VisionTest extends OpMode {
    Cameras cameras;

    @Override
    public void init() {
        cameras = new Cameras(hardwareMap);
    }


    @Override
    public void init_loop() {
        loop();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            cameras.barcodeProcessor.setSide(RobotSide.Blue);
        } else if (gamepad1.b) {
            cameras.barcodeProcessor.setSide(RobotSide.Red);
        }
        telemetry.addData("Current side, use A and B to switch", cameras.barcodeProcessor.side);
        telemetry.addData("Barcode Position", cameras.getBarcodePosition());
        // Pose2d finalUpdatePose = new Pose2d(0, 0, 0);
        // drivetrain.setPoseEstimate(finalUpdatePose)
        telemetry.update();
    }
}