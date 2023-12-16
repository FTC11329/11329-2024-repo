package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Cameras;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.utility.BarcodePosition;
import org.firstinspires.ftc.teamcode.utility.RobotSide;

@Autonomous(name = "Red Left", group = "Competition")
@Config
public class RedLeft extends LinearOpMode {
    static Pose2d startingPose = new Pose2d(-41, -60, Math.toRadians(90));
    static Vector2d placePositionOne = new Vector2d(53, -38);
    static Vector2d placePositionTwo = new Vector2d(53, -38);
    static Vector2d placePositionThree = new Vector2d(50, -45);

    @Override
    public void runOpMode() throws InterruptedException {
        Cameras cameras = new Cameras(hardwareMap);
        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        cameras.barcodeProcessor.setSide(RobotSide.Red);

        waitForStart();

        BarcodePosition barcodePosition = cameras.getBarcodePosition().orElse(BarcodePosition.One);

        drivetrain.setPoseEstimate(startingPose);

        TrajectorySequence placeSpikeMark = null;

        if (barcodePosition == BarcodePosition.Three) {
            placeSpikeMark = drivetrain.trajectorySequenceBuilder(startingPose)
                    .splineTo(new Vector2d(-31, -38), Math.toRadians(45))
                    .addTemporalMarker(() -> {
                        intake.setIntakePower(Constants.Intake.autoVomitSpeed, 10);
                    })
                    .addTemporalMarkerOffset(0.1, () -> {
                        intake.setIntakePower(0, 10);
                    })
                    .waitSeconds(0.5)
                    .back(15)
                    .setReversed(true)
                    .splineTo(new Vector2d(-34, -9), Math.toRadians(0))
                    .build();

        } else if (barcodePosition == BarcodePosition.Two) {
            placeSpikeMark = drivetrain.trajectorySequenceBuilder(startingPose)
                    .splineTo(new Vector2d(-41, -30), Math.toRadians(90))
                    .addTemporalMarker(() -> {
                        intake.setIntakePower(Constants.Intake.autoVomitSpeed, 10);
                    })
                    .addTemporalMarkerOffset(0.05, () -> {
                        intake.setIntakePower(0, 10);
                    })
                    .waitSeconds(0.5)
                    .lineTo(new Vector2d(-53, -35))
                    .setReversed(true)
                    .splineTo(new Vector2d(-59, -25), Math.toRadians(90))
                    .splineTo(new Vector2d(-34, -9), Math.toRadians(0))
                    .build();

        } else if (barcodePosition == BarcodePosition.One) {
            placeSpikeMark = drivetrain.trajectorySequenceBuilder(startingPose)
                    .lineTo(new Vector2d(-50, -37))
                    .addTemporalMarker(() -> {
                        intake.setIntakePower(Constants.Intake.autoVomitSpeed, 10);
                    })
                    .addTemporalMarkerOffset(0.05, () -> {
                        intake.setIntakePower(0, 10);
                    })
                    .waitSeconds(0.5)
                    .lineToLinearHeading(new Pose2d(-54, -52, Math.toRadians(90)))
                    .setReversed(true)
                    .splineTo(new Vector2d(-50, -10), Math.toRadians(0))
                    .splineTo(new Vector2d(-34, -9), Math.toRadians(0))
                    .build();
        }

        drivetrain.followTrajectorySequence(placeSpikeMark);

        Vector2d finalPlaceLocation = null;

        if (barcodePosition == BarcodePosition.One) {
            finalPlaceLocation = placePositionOne;
        } else if (barcodePosition == BarcodePosition.Two) {
            finalPlaceLocation = placePositionTwo;
        } else if (barcodePosition == BarcodePosition.Three) {
            finalPlaceLocation = placePositionThree;
        } else return;

        drivetrain.followTrajectorySequence(drivetrain
                .trajectorySequenceBuilder(placeSpikeMark.end())
                .setReversed(true)
                .splineTo(new Vector2d(-11, -9), Math.toRadians(0))
                .splineTo(finalPlaceLocation.plus(new Vector2d(-10, 0)), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    outtake.preset(Constants.Slides.low, Constants.Arm.placePos);
                }).waitSeconds(1)
                .lineTo(finalPlaceLocation)
                .addTemporalMarker(() -> {
                    claw.setPower(Constants.Claw.outake);
                })
                .build());
    }
}
