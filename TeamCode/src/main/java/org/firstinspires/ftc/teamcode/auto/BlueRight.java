package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.utility.BarcodePosition;

@Autonomous(name = "Blue Right", group = "Competition")
@Config
public class BlueRight extends LinearOpMode {
    static Pose2d startingPose = new Pose2d(-41, 60, Math.toRadians(-90));
    static Vector2d placePositionOne = new Vector2d(48, 36);
    static Vector2d placePositionTwo = new Vector2d();
    static Vector2d placePositionThree = new Vector2d();

    BarcodePosition barcodePosition = BarcodePosition.Two;

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        waitForStart();

        drivetrain.setPoseEstimate(startingPose);

        TrajectorySequence placeSpikeMark = null;

        if (barcodePosition == BarcodePosition.One) {
            placeSpikeMark = drivetrain.trajectorySequenceBuilder(startingPose)
                    .splineTo(new Vector2d(-30, 38), Math.toRadians(-46))
                    .addTemporalMarker(() -> {
                        intake.setIntakePower(Constants.Intake.autoVomitSpeed);
                    })
                    .addTemporalMarkerOffset(0.1, () -> {
                        intake.setIntakePower(0);
                    })
                    .waitSeconds(1)
                    .back(15)
                    .splineTo(new Vector2d(-34, 9), Math.toRadians(180))
                    .build();

        } else if (barcodePosition == BarcodePosition.Two) {
            placeSpikeMark = drivetrain.trajectorySequenceBuilder(startingPose)
                    .splineTo(new Vector2d(-41,30), Math.toRadians(-90))
                    .addTemporalMarker(() -> {
                        intake.setIntakePower(Constants.Intake.autoVomitSpeed);
                    })
                    .addTemporalMarkerOffset(0.05, () -> {
                        intake.setIntakePower(0);
                    })
                    .waitSeconds(0.5)
                    .splineTo(new Vector2d(-50,32), Math.toRadians(-90))
                    .splineTo(new Vector2d(-34, 9), Math.toRadians(180))
                    .build();

        } else if (barcodePosition == BarcodePosition.Three) {
            placeSpikeMark = drivetrain.trajectorySequenceBuilder(startingPose).build();
        }

        drivetrain.followTrajectorySequence(placeSpikeMark);

        Vector2d finalPlaceLocation = null;

        if (barcodePosition == BarcodePosition.One) {
            finalPlaceLocation = placePositionOne;
        } else if (barcodePosition == BarcodePosition.Two) {
            finalPlaceLocation = placePositionTwo;
        } else if (barcodePosition == BarcodePosition.Three) {
            finalPlaceLocation = placePositionThree;
        }

        drivetrain.followTrajectorySequence(drivetrain
                .trajectorySequenceBuilder(placeSpikeMark.end())
                .splineTo(new Vector2d(-11, 4), Math.toRadians(0))
                .splineTo(finalPlaceLocation, Math.toRadians(0))
                .addTemporalMarker(() -> {
                    outtake.preset(Constants.Slides.low, Constants.Arm.placePos);
                }).waitSeconds(1)
                .back(7)
                .addTemporalMarker(() -> {
                    claw.setPower(Constants.Claw.outake);
                })
                .build());
    }
}
