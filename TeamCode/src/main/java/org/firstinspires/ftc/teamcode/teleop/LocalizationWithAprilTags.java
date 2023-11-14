package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@TeleOp(name = "Allen Test Drive", group = "Allen op mode")
public class LocalizationWithAprilTags extends OpMode {
    Drivetrain drivetrain;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline();
    }

    @Override
    public void loop() {
        drivetrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, DriveSpeedEnum.Fast);
        if (aprilTagDetectionPipeline.getDesiredTag(1).isPresent()) {
            AprilTagPoseFtc tagPose = aprilTagDetectionPipeline.getDesiredTag(1).get().ftcPose;


            telemetry.addData("x", tagPose.x);
            telemetry.addData("y", tagPose.y);
            telemetry.addData("yaw", tagPose.yaw);
            // Pose2d finalUpdatePose = new Pose2d(0, 0, 0);
            // drivetrain.setPoseEstimate(finalUpdatePose);
        }
    }
}
