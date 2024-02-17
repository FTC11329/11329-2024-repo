package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Cameras;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.SpecialIntake;
import org.firstinspires.ftc.teamcode.utility.BarcodePosition;

import java.util.Optional;
@Disabled
@Autonomous(name = "Red Left 2 + 4 NOOOOOOOOOOO Run", group = "Competition")
@Config
public class RedLeft4 extends LinearOpMode {
    static Pose2d startingPose = new Pose2d(-41, -60, Math.toRadians(90));
    static Vector2d placePositionOne   = new Vector2d(52, -31);
    static Vector2d placePositionTwo   = new Vector2d(52, -39);
    static Vector2d placePositionThree = new Vector2d(52, -45);

    static Vector2d pickupSpecial = new Vector2d(-55.5,-11);
    static Vector2d pickupSpecial2 = new Vector2d(-53.5, -19);


    static double timeForPixelPlacement = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Cameras cameras = new Cameras(hardwareMap);
        SpecialIntake specialIntake = new SpecialIntake(hardwareMap);
        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        DistanceSensors distanceSensors = new DistanceSensors(hardwareMap);

        while (!opModeIsActive() && !isStopRequested()) {
            BarcodePosition barcodePosition = distanceSensors.getDirectionRed();
            telemetry.addData("Barcode Position", barcodePosition);
            telemetry.update();
        }
        waitForStart();

        BarcodePosition barcodePosition = distanceSensors.getDirectionRed();

        drivetrain.setPoseEstimate(startingPose);

        TrajectorySequence placeSpikeMark = null;

        if (barcodePosition == BarcodePosition.One) {
            placeSpikeMark = drivetrain.trajectorySequenceBuilder(startingPose)
                    .addTemporalMarker(() -> {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.down3);
                        intake.setIntakePower(-0.5, 0);
                    })
                    .splineTo(new Vector2d(-47, -15), Math.toRadians(90))
                    .addTemporalMarker(() -> {
                        outtake.presetArm(Constants.Arm.autoArmDrop);
                    })
                    .addTemporalMarkerOffset(timeForPixelPlacement, () -> {
                        outtake.presetArm(Constants.Arm.intakePos);
                        specialIntake.setIntakeServo(Constants.SpecialIntake.up);
                        intake.setIntakePower(0, 0);
                    })
                    .waitSeconds(timeForPixelPlacement)
                    .lineToLinearHeading(new Pose2d(-45,-7, Math.toRadians(180)))
                    .build();

        } else if (barcodePosition == BarcodePosition.Two) {
            placeSpikeMark = drivetrain.trajectorySequenceBuilder(startingPose)
                    .addTemporalMarker(() -> {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.down3);
                        intake.setIntakePower(-0.5, 0);
                    })
                    .lineTo(new Vector2d(-40, -11))
                    .addTemporalMarker(() -> {
                        outtake.presetArm(Constants.Arm.autoArmDrop);
                    })
                    .addTemporalMarkerOffset(timeForPixelPlacement, () -> {
                        outtake.presetArm(Constants.Arm.intakePos);
                        specialIntake.setIntakeServo(Constants.SpecialIntake.up);
                        intake.setIntakePower(0, 0);
                    })
                    .waitSeconds(timeForPixelPlacement)
                    .lineToLinearHeading(new Pose2d(-45,-7, Math.toRadians(180)))
                    .build();

        } else if (barcodePosition == BarcodePosition.Three) {
            placeSpikeMark = drivetrain.trajectorySequenceBuilder(startingPose)
                    .lineToLinearHeading(new Pose2d(-36, -31, Math.toRadians(180)))
                    .addTemporalMarker(() -> {
                        outtake.presetArm(Constants.Arm.autoArmDrop);
                    })
                    .addTemporalMarkerOffset(timeForPixelPlacement, () -> {
                        outtake.presetArm(Constants.Arm.intakePos);
                        specialIntake.setIntakeServo(Constants.SpecialIntake.up);
                    })
                    .waitSeconds(timeForPixelPlacement)
                    .lineToLinearHeading(new Pose2d(-45,-7, Math.toRadians(180)))
                    .build();
        }

        drivetrain.followTrajectorySequence(placeSpikeMark);

        Vector2d finalPlaceLocation = null;

        if (barcodePosition == BarcodePosition.One) {
            finalPlaceLocation = placePositionOne;
        } else if (barcodePosition == BarcodePosition.Two) {
            finalPlaceLocation = placePositionTwo;
        } else {
            finalPlaceLocation = placePositionThree;
        }


        drivetrain.followTrajectorySequence(drivetrain
                .trajectorySequenceBuilder(placeSpikeMark.end())
                .setReversed(true)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .splineTo(new Vector2d(-29,-6), Math.toRadians(0))
                .splineTo(new Vector2d(-11, -7), Math.toRadians(0))
                .splineTo(new Vector2d(25, -11), Math.toRadians(0))
                .resetConstraints()
                .addTemporalMarker(() -> {
                    outtake.preset(Constants.Slides.superLow, Constants.Arm.placePos);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(4, true);
                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
                })
                .lineTo(finalPlaceLocation)
                .addTemporalMarkerOffset(0.1,() -> {
                    claw.setPower(Constants.Claw.outake);
                })
                .addTemporalMarkerOffset(2, () -> {
                    outtake.preset(Constants.Slides.intake, Constants.Arm.intakePos);
                    claw.setPower(0);
                })
                .waitSeconds(0.5)
                //Back For Another One**************************************************************
                .setReversed(false)
                .splineTo(new Vector2d(36, -12), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(4, true);
                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
                })
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 40, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 40  //acc
                )
                .splineTo(pickupSpecial2, Math.toRadians(180))

                .addTemporalMarkerOffset(-1, () -> {
                    intake.setIntakePower(Constants.Intake.intake, 0);
                    claw.setPower(Constants.Claw.intake);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                })
                .addTemporalMarkerOffset(0, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down5);
                })
                .addTemporalMarkerOffset(0.5, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down4);
                })
                .addTemporalMarkerOffset(1, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.up);
                })
                .waitSeconds(1)
                .setReversed(true)
                .addTemporalMarkerOffset(0.75, () -> {
                    intake.setIntakePower(Constants.Intake.outake, 0);
                    claw.setPower(0);
                })
                .addTemporalMarkerOffset(2, () -> {
                    intake.setIntakePower(0, 0);
                })
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .splineTo(new Vector2d(-11, -12), Math.toRadians(0))
                .splineTo(new Vector2d(25, -11), Math.toRadians(0))
                .resetConstraints()
                .addTemporalMarker(() -> {
                    outtake.preset(Constants.Slides.low, Constants.Arm.placePos);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(4, true);
                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
                })
                .lineTo(placePositionOne)
                .addTemporalMarker(() -> {
                    claw.setPower(Constants.Claw.outake);
                })
                .waitSeconds(0.5)
                //ANOTHER ONE***********************************************************************
                .addTemporalMarkerOffset(2, () -> {
                    outtake.preset(Constants.Slides.intake, Constants.Arm.intakePos);
                    claw.setPower(0);
                })
                .setReversed(false)
                .splineTo(new Vector2d(36, -12), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(4, true);
                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
                })
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 40, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 40  //acc
                )
                .splineTo(pickupSpecial2, Math.toRadians(180))

                .addTemporalMarkerOffset(-1, () -> {
                    intake.setIntakePower(Constants.Intake.intake, 0);
                    claw.setPower(Constants.Claw.intake);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                })
                .addTemporalMarkerOffset(0, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down3);
                })
                .addTemporalMarkerOffset(0.5, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down2);
                })
                .addTemporalMarkerOffset(1, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.up);
                })
                .waitSeconds(1)
                .setReversed(true)
                .addTemporalMarkerOffset(0.75, () -> {
                    intake.setIntakePower(Constants.Intake.outake, 0);
                    claw.setPower(0);
                })
                .addTemporalMarkerOffset(2, () -> {
                    intake.setIntakePower(0, 0);
                })
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .splineTo(new Vector2d(-11, -12), Math.toRadians(0))
                .splineTo(new Vector2d(25, -11), Math.toRadians(0))
                .resetConstraints()
                .addTemporalMarker(() -> {
                    outtake.preset(Constants.Slides.low, Constants.Arm.placePos);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(4, true);
                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
                })
                .lineTo(placePositionOne)
                .addTemporalMarker(() -> {
                    claw.setPower(Constants.Claw.outake);
                })
                .waitSeconds(0.5)
                .build());
    }
}
