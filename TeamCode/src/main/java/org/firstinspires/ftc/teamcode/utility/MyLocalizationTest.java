package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Cameras;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;

import java.util.Optional;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(name = "My Localization Test", group = "Allen op mode")
public class MyLocalizationTest extends LinearOpMode {
    Drivetrain drive;
    AprilTagToRoadRunner aprilTagToRoadRunner;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    Cameras cameras;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain(hardwareMap, telemetry);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(new Pose2d(gamepad1.left_stick_y * 0.7, gamepad1.left_stick_x * 0.7, gamepad1.right_stick_x * 0.7));

            drive.update();
            Optional<Pose2d> optionalDetection = cameras.getRunnerPoseEstimate(10);
            optionalDetection.ifPresent(pose2d -> drive.setPoseEstimate(pose2d));

            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
