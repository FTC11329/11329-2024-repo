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
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.utility.BarcodePosition;
import org.firstinspires.ftc.teamcode.utility.RobotSide;

@Autonomous(name = "Blue Right", group = "Competition")
@Config
public class BlueRight extends LinearOpMode {
    static Pose2d startingPose = new Pose2d(-41, 60, Math.toRadians(-90));
    static Vector2d placePositionOne = new Vector2d(56, 26);
    static Vector2d placePositionTwo = placePositionOne;
    static Vector2d placePositionThree = placePositionOne;

    static double timeForPixelPlacement = 0.02;

    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Cameras cameras = new Cameras(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        DistanceSensors distanceSensors = new DistanceSensors(hardwareMap);


        waitForStart();

        BarcodePosition barcodePosition = distanceSensors.getDirectionBlue();

        barcodePosition = BarcodePosition.Three;
        drivetrain.setPoseEstimate(startingPose);

        TrajectorySequence placeSpikeMark = null;

        if (barcodePosition == BarcodePosition.One) {
            placeSpikeMark = drivetrain.trajectorySequenceBuilder(startingPose)
                    .splineTo(new Vector2d(-29, 38), Math.toRadians(-45))
                    .addTemporalMarker(() -> {
                        outtake.presetArm(Constants.Arm.autoArmDrop);
                    })
                    .addTemporalMarkerOffset(timeForPixelPlacement, () -> {
                        outtake.presetArm(0);
                    })
                    .waitSeconds(3 * timeForPixelPlacement)
                    .back(15)
                    .setReversed(true)
                    .splineTo(new Vector2d(-32, 9), Math.toRadians(0))
                    .build();

        } else if (barcodePosition == BarcodePosition.Two) {
            placeSpikeMark = drivetrain.trajectorySequenceBuilder(startingPose)
                    .splineTo(new Vector2d(-41, 30), Math.toRadians(-90))
                    .addTemporalMarker(() -> {
                        outtake.presetArm(Constants.Arm.autoArmDrop);
                    })
                    .addTemporalMarkerOffset(timeForPixelPlacement, () -> {
                        outtake.presetArm(0);
                    })
                    .waitSeconds(3 * timeForPixelPlacement)
                    .back(5)
                    .setReversed(true)
                    .splineTo(new Vector2d(-50, 32), Math.toRadians(-90))
                    .splineTo(new Vector2d(-32, 9), Math.toRadians(0))
                    .build();

        } else if (barcodePosition == BarcodePosition.Three) {
            placeSpikeMark = drivetrain.trajectorySequenceBuilder(startingPose)
                    .lineTo(new Vector2d(-48, 21))
                    .addTemporalMarker(() -> {
                        outtake.presetArm(Constants.Arm.autoArmDrop);
                    })
                    .addTemporalMarkerOffset(timeForPixelPlacement, () -> {
                        outtake.presetArm(0);
                    })
                    .waitSeconds(3 * timeForPixelPlacement)
                    .turn(Math.toRadians(-40))
                    .splineTo(new Vector2d(-54, 5), Math.toRadians(-110))
                    .turn(Math.toRadians(-50))
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
                .splineTo(new Vector2d(-11, 9), Math.toRadians(0))
                .splineTo(new Vector2d(25, 9), Math.toRadians(0))
                .splineTo(finalPlaceLocation.plus(new Vector2d(-10, 0)), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    outtake.preset(Constants.Slides.superLow, Constants.Arm.placePos);
                })
                .waitSeconds(0.05)
                .lineTo(finalPlaceLocation)
                .addTemporalMarker(() -> {
                    claw.setPower(Constants.Claw.outake);
                })
                .waitSeconds(0.75)
                .lineTo(finalPlaceLocation.plus(new Vector2d(-10, 0)))
                .addTemporalMarker(() -> {
                    outtake.preset(Constants.Slides.intake, 0.1);
                })
                .waitSeconds(2)
                .build());
    }
}
