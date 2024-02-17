package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.C;
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

@Autonomous(name = "Red Left 3 + 2", group = " Editing")
@Config
public class RedLeft extends LinearOpMode {
    static Pose2d startingPose = new Pose2d(-41, -60, Math.toRadians(90));
    static Vector2d placePositionOne   = new Vector2d(52, -31);
    static Vector2d placePositionTwo   = new Vector2d(52, -38.25);
    static Vector2d placePositionThree = new Vector2d(52, -42);

    static Vector2d pickupSpecial = new Vector2d(-55,-11);
    static Vector2d pickupSpecial2 = new Vector2d(-57, -8);


    static double timeForPixelPlacement = 0.15;

    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Cameras cameras = new Cameras(hardwareMap);
        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        SpecialIntake specialIntake = new SpecialIntake(hardwareMap);
        DistanceSensors distanceSensors = new DistanceSensors(hardwareMap);

        while (!opModeIsActive() && !isStopRequested()) {
            BarcodePosition barcodePosition = distanceSensors.getDirectionRed();
            telemetry.addData("Barcode Position", barcodePosition);
            telemetry.addData("FPS Back" , cameras.backCamera.getFps());
            telemetry.addData("FPS Front", cameras.frontCamera.getFps());
            telemetry.update();
        }
        waitForStart();

        BarcodePosition barcodePosition = distanceSensors.getDirectionRed();

        drivetrain.setPoseEstimate(startingPose);

        TrajectorySequence placeSpikeMark = null;

        if (barcodePosition == BarcodePosition.One) {
            placeSpikeMark = drivetrain.trajectorySequenceBuilder(startingPose)
                    .setConstraints(
                            (displacement, pose, derivative, baseRobotVelocity) -> 50, //vel
                            (displacement, pose, derivative, baseRobotVelocity) -> 50  //acc
                    )
                    .addTemporalMarker(() -> {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.down3);
                        intake.setIntakePower(-0.5, 0);
                    })
                    .splineTo(new Vector2d(-48, -14), Math.toRadians(90))
                    .addTemporalMarker(() -> {
                        outtake.presetArm(Constants.Arm.autoArmDrop);
                    })
                    .addTemporalMarkerOffset(timeForPixelPlacement, () -> {
                        outtake.presetArm(Constants.Arm.intakePos);
                        specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                        intake.setIntakePower(0, 0);
                    })
                    .waitSeconds(timeForPixelPlacement)
                    .splineTo(pickupSpecial, Math.toRadians(180))
                    .build();

        } else if (barcodePosition == BarcodePosition.Two) {
            placeSpikeMark = drivetrain.trajectorySequenceBuilder(startingPose)
                    .setConstraints(
                            (displacement, pose, derivative, baseRobotVelocity) -> 50, //vel
                            (displacement, pose, derivative, baseRobotVelocity) -> 50  //acc
                    )
                    .addTemporalMarker(() -> {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.down3);
                        intake.setIntakePower(-0.5, 0);
                    })
                    .lineTo(new Vector2d(-35, -11))
                    .addTemporalMarker(() -> {
                        outtake.presetArm(Constants.Arm.autoArmDrop);
                    })
                    .addTemporalMarkerOffset(timeForPixelPlacement, () -> {
                        outtake.presetArm(Constants.Arm.intakePos);
                        specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                        intake.setIntakePower(0, 0);
                    })
                    .waitSeconds(timeForPixelPlacement)
                    .splineTo(pickupSpecial, Math.toRadians(180))
                    .build();

        } else if (barcodePosition == BarcodePosition.Three) {
            placeSpikeMark = drivetrain.trajectorySequenceBuilder(startingPose)
                    .setConstraints(
                            (displacement, pose, derivative, baseRobotVelocity) -> 50, //vel
                            (displacement, pose, derivative, baseRobotVelocity) -> 50  //acc
                    )
                    .lineToLinearHeading(new Pose2d(-36, -31, Math.toRadians(180)))
                    .addTemporalMarker(() -> {
                        outtake.presetArm(Constants.Arm.autoArmDrop);
                    })
                    .addTemporalMarkerOffset(timeForPixelPlacement, () -> {
                        outtake.presetArm(Constants.Arm.intakePos);
                        specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                    })
                    .waitSeconds(timeForPixelPlacement)
                    .lineTo(pickupSpecial.plus(new Vector2d(-2,-3)))
                    .build();
        }

        drivetrain.followTrajectorySequence(placeSpikeMark);

        Vector2d finalPlaceLocation = null;
        Vector2d finalPlaceLocation2 = null;

        if (barcodePosition == BarcodePosition.One) {
            finalPlaceLocation = placePositionOne;
            finalPlaceLocation2 = placePositionTwo;
        } else if (barcodePosition == BarcodePosition.Two) {
            finalPlaceLocation = placePositionTwo;
            finalPlaceLocation2 = placePositionOne;
        } else if (barcodePosition == BarcodePosition.Three) {
            finalPlaceLocation = placePositionThree;
            finalPlaceLocation2 = placePositionOne;
        } else return;



        drivetrain.followTrajectorySequence(drivetrain
                .trajectorySequenceBuilder(placeSpikeMark.end())
                .resetConstraints()
                .setReversed(true)
                .addTemporalMarkerOffset(-1, () -> {
                    outtake.presetArm(Constants.Arm.intakePos);
                    intake.setIntakePower(Constants.Intake.intake, 0);
                    claw.setPower(Constants.Claw.intake);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                })
                .addTemporalMarkerOffset(0, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down5);
                })
                .addTemporalMarkerOffset(0.75, () -> {
                    intake.setIntakePower(Constants.Intake.intake, 0);
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
                .splineTo(new Vector2d(-11, -7), Math.toRadians(0))
                .splineTo(new Vector2d(25, -5), Math.toRadians(0))
                .splineTo(new Vector2d(37, -21), Math.toRadians(350))
                .addTemporalMarker(() -> {
                    outtake.preset(Constants.Slides.superLow, Constants.Arm.placePos);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, true);
                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
                    telemetry.addData("did see one", optionalPose.isPresent());
                    telemetry.update();
                })
                .resetConstraints()
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 25, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 25  //acc
                )
                .lineToLinearHeading(new Pose2d(finalPlaceLocation.getX(), finalPlaceLocation.getY(), Math.toRadians(180)))
                .resetConstraints()
                .addTemporalMarkerOffset(0.1,() -> {
                    claw.setPower(Constants.Claw.outake);
                })
                .addTemporalMarkerOffset(0.5, () -> {
                    outtake.presetSlides(Constants.Slides.low);
                })
                .addTemporalMarkerOffset(2, () -> {
                    outtake.preset(Constants.Slides.intake, 0);
                    claw.setPower(0);
                })
                .waitSeconds(0.5)
                .setReversed(false)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .splineTo(new Vector2d(36, -7), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-30, -7, Math.toRadians(210)))
                .waitSeconds(0.25)
                .addTemporalMarkerOffset(0, () -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, false);
                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
                    telemetry.addData("did see two", optionalPose.isPresent());
                    telemetry.update();
                })

                //Back For Another One**************************************

                .resetConstraints()
                .lineToLinearHeading(new Pose2d(pickupSpecial2.getX(), pickupSpecial2.getY(), Math.toRadians(180)))

                .addTemporalMarkerOffset(-1, () -> {
                    intake.setIntakePower(Constants.Intake.intake, 0);
                    claw.setPower(Constants.Claw.intake);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                })
                .addTemporalMarkerOffset(0, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down4);
                })
                .addTemporalMarkerOffset(0.5, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down3);
                })
                .addTemporalMarkerOffset(1, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.up);
                })
                .waitSeconds(1)
                .forward(2.5)
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
                .splineTo(new Vector2d(25, -10), Math.toRadians(0))
                .splineTo(new Vector2d(30, -31), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    outtake.preset(Constants.Slides.med - 600, Constants.Arm.placePos);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, true);
                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
                    telemetry.addData("did see Three", optionalPose.isPresent());
                    telemetry.update();
                })
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 35, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 35  //acc
                )
                .resetConstraints()
                .lineTo(finalPlaceLocation2)
                .addTemporalMarker(() -> {
                    claw.setPower(Constants.Claw.outake);
                })
                .waitSeconds(0.5)
                .forward(10)
                .addTemporalMarkerOffset(-0.5, () -> {
                    claw.setPower(0);
                    outtake.presetSlides(Constants.Slides.intake);
                    outtake.presetArm(Constants.Arm.intakePos);
                })
                .build());
    }
}
