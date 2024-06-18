package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Cameras;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utility.AprilTagToRoadRunner;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

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
    Cameras cameras;
    double temp;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain(hardwareMap);
        cameras = new Cameras(hardwareMap);
        Servo frontClawServo = hardwareMap.get(Servo.class, "clawServoF");

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontClawServo.setDirection(Servo.Direction.REVERSE);


        waitForStart();

        while (!isStopRequested()) {
            boolean isBack = gamepad1.y;
            drive.setWeightedDrivePower(new Pose2d(gamepad1.left_stick_y * -0.7, gamepad1.left_stick_x * -0.7, gamepad1.right_stick_x * -0.7));

            drive.update();
            frontClawServo.setPosition(temp);
            temp+= (gamepad1.right_trigger- gamepad1.left_trigger)*0.005;

            Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, isBack);
            optionalPose.ifPresent(pose2d -> drive.setPoseEstimate(pose2d));

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("temp", temp);
            telemetry.addData("is back", isBack);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("seas tag", optionalPose.isPresent());

            telemetry.update();
        }
    }
}
