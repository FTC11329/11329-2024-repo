package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AutoServo;
import org.firstinspires.ftc.teamcode.subsystems.Cameras;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ClawSensor;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.SpecialIntake;
import org.firstinspires.ftc.teamcode.utility.BarcodePosition;

import java.util.Optional;

@Autonomous(name = "Auto test", group = " test")
@Config
public class OtosAuto extends OpMode {
    static Pose2d startingPose = new Pose2d(17, -64, Math.toRadians(90));
    TrajectorySequence testPath = null;


    Intake intake;
    Outtake outtake;
    Cameras cameras;
    ClawSensor clawSensor;
    Drivetrain drivetrain;
    SpecialIntake specialIntake;
    DistanceSensors distanceSensors;

    public void init() {
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        cameras = new Cameras(hardwareMap);
        clawSensor = new ClawSensor(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        specialIntake = new SpecialIntake(hardwareMap);
        distanceSensors = new DistanceSensors(hardwareMap);
        outtake.holdClaw(true);
    }

    @Override
    public void init_loop() {
        boolean isBack = gamepad1.a;
        cameras.setCameraSide(gamepad1.a);

        BarcodePosition barcodePosition = distanceSensors.getDirectionRed(false);
        telemetry.addData("Barcode Position", barcodePosition);
        telemetry.addData("FPS", cameras.switchingCamera.getFps());
        telemetry.addData("Is back", isBack);
        telemetry.update();
    }

    @Override
    public void start() {
        cameras.setCameraSideThreaded(true);
        BarcodePosition barcodePosition = distanceSensors.getDirectionRed(false);

        drivetrain.setPoseEstimateOptical(startingPose);

        drivetrain.followTrajectorySequenceAsync(drivetrain
                .trajectorySequenceBuilder(startingPose)
                .splineTo(new Vector2d(28.6, -18.7), Math.toRadians(90))
                .setReversed(true)
                .splineTo(new Vector2d(9.7, -67.9), Math.toRadians(45))
                .setReversed(false)

                .splineTo(new Vector2d(28.6, -18.7), Math.toRadians(90))
                .setReversed(true)
                .splineTo(new Vector2d(9.7, -67.9), Math.toRadians(45))
                .setReversed(false)

                .splineTo(new Vector2d(28.6, -18.7), Math.toRadians(90))
                .setReversed(true)
                .splineTo(new Vector2d(9.7, -67.9), Math.toRadians(45))
                .setReversed(false)

                .build());
    }

    @Override
    public void loop() {
        drivetrain.update();
        outtake.periodic();
    }
}