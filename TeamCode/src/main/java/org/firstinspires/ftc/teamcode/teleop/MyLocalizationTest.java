package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Cameras;
import org.firstinspires.ftc.teamcode.subsystems.ClawSensor;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.SpecialIntake;

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
    Lights lights;
    Outtake outtake;
    Cameras cameras;
    Drivetrain drivetrain;
    SpecialIntake specialIntake;
    boolean picDe = false;
    boolean see;
    int id;
    boolean idRDe = false;
    boolean idLDe = false;

    @Override
    public void runOpMode() throws InterruptedException {
        lights = new Lights(hardwareMap);
        outtake = new Outtake(hardwareMap);
        cameras = new Cameras(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        specialIntake = new SpecialIntake(hardwareMap);

        drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {

            boolean isBack = gamepad1.y;
            drivetrain.setWeightedDrivePower(new Pose2d(gamepad1.left_stick_y * -0.5, gamepad1.left_stick_x * -0.5, gamepad1.right_stick_x * -0.5));

            drivetrain.update();

            if (gamepad1.b && picDe) {
                Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(id, isBack);
                optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
                see = optionalPose.isPresent();
                picDe = false;
            }
            if (!gamepad1.b) {
                picDe = true;
            }

            if (gamepad1.dpad_up) {
                outtake.createPresetThread(Constants.Slides.med, Constants.Arm.placePos, 5, true, true);
            } else if (gamepad1.dpad_down) {
                outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false, false);
            }

            Pose2d poseEstimate = drivetrain.getPoseEstimate();
            telemetry.addData("is back", isBack);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("id", id);
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("seas tag", see);

            telemetry.update();
        }
    }
}
