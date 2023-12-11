package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Cameras;
import org.firstinspires.ftc.teamcode.subsystems.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.Optional;

@TeleOp(name = "Allen Test Drive", group = "Allen op mode")
public class LocalizationWithAprilTags extends OpMode {
    Drivetrain drivetrain;
    Cameras cameras;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        cameras = new Cameras(hardwareMap);
    }

    @Override
    public void loop() {
        drivetrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, DriveSpeedEnum.Fast);

        Optional<Pose2d> poseOptional = cameras.getPoseEstimate();

        poseOptional.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));

        telemetry.addData("Pose", drivetrain.getPoseEstimate());
    }
}
