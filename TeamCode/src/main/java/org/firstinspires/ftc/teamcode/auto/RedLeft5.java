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
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.SpecialIntake;
import org.firstinspires.ftc.teamcode.utility.BarcodePosition;

import java.util.Optional;

@Autonomous(name = "Red Left 5 + 2", group = " Testing")
@Config
public class RedLeft5 extends OpMode {
    boolean goLeft;
    boolean hasTwo;
    static Pose2d startingPose = new Pose2d(-41, -60, Math.toRadians(90));
    static Vector2d placePositionOne = new Vector2d(52.5, -31.5);
    static Vector2d placePositionTwo = new Vector2d(52, -38.25);
    static Vector2d placePositionThree = new Vector2d(52, -43.75);

    static Vector2d placePositionLeft = new Vector2d(52.5, -35);

    static Vector2d pickupSpecial = new Vector2d(-56, -12);
    static Vector2d pickupSpecial2 = new Vector2d(-57, -6);

    static double timeForPixelPlacement = 0.15;

    Claw claw;
    Intake intake;
    Outtake outtake;
    Cameras cameras;
    AutoServo autoServo;
    ClawSensor clawSensor;
    Drivetrain drivetrain;
    SpecialIntake specialIntake;
    DistanceSensors distanceSensors;

    public void init() {
        claw = new Claw(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        cameras = new Cameras(hardwareMap);
        autoServo = new AutoServo(hardwareMap);
        clawSensor = new ClawSensor(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        specialIntake = new SpecialIntake(hardwareMap);
        distanceSensors = new DistanceSensors(hardwareMap);
    }

    @Override
    public void init_loop() {
        boolean isBack = gamepad1.a;
        cameras.setCameraSide(gamepad1.a);

        BarcodePosition barcodePosition = distanceSensors.getDirectionRed(true);
        telemetry.addData("Barcode Position", barcodePosition);
        telemetry.addData("FPS", cameras.switchingCamera.getFps());
        telemetry.addData("Is back", isBack);
        telemetry.update();
    }

    @Override
    public void start() {
        cameras.setCameraSide(true);
        BarcodePosition barcodePosition = distanceSensors.getDirectionRed(true);

        drivetrain.setPoseEstimate(startingPose);

        TrajectorySequence placeSpikeMark = null;

        if (barcodePosition == BarcodePosition.One) {
            placeSpikeMark = drivetrain.trajectorySequenceBuilder(startingPose)
                    .addTemporalMarker(() -> {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.down3);
                        intake.setIntakePower(-0.5, 0);
                    })
                    .lineTo(new Vector2d(-49, -50))
                    .lineTo(new Vector2d(-47, -16))
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
                        specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                        intake.setIntakePower(0, 0);
                    })
                    .waitSeconds(timeForPixelPlacement)
                    .splineTo(pickupSpecial, Math.toRadians(180))
                    .build();

        } else if (barcodePosition == BarcodePosition.Three) {
            placeSpikeMark = drivetrain.trajectorySequenceBuilder(startingPose)
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
//            finalPlaceLocation = placePositionLeft;
            finalPlaceLocation  = placePositionOne;
            finalPlaceLocation2 = placePositionTwo;
            goLeft = false;
        } else if (barcodePosition == BarcodePosition.Two) {
            finalPlaceLocation  = placePositionTwo;
            finalPlaceLocation2 = placePositionOne;
            goLeft = true;
        } else if (barcodePosition == BarcodePosition.Three) {
            finalPlaceLocation  = placePositionThree;
            finalPlaceLocation2 = placePositionOne.plus(new Vector2d(-1,0));
            goLeft = false;
        } else return;


        drivetrain.followTrajectorySequence(drivetrain
                .trajectorySequenceBuilder(placeSpikeMark.end())
                .resetConstraints()
                .setReversed(true)
                .addTemporalMarkerOffset(0, () -> {
                    outtake.presetArm(Constants.Arm.intakePos);
                    intake.setIntakePower(Constants.Intake.intake, 0);
                    claw.setPower(Constants.Claw.intake);
                })
                .addTemporalMarkerOffset(0.1, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down5);
                })
                .addTemporalMarkerOffset(0.6, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.up);
                })
                .waitSeconds(0.6)
                .setReversed(true)
                .addTemporalMarkerOffset(0.5, () -> {
                    intake.setIntakePower(Constants.Intake.outake, 0);
                    claw.setPower(0);
                })
                .addTemporalMarkerOffset(2, () -> {
                    intake.setIntakePower(0, 0);
                    hasTwo = clawSensor.isFull();
                })
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .splineTo(new Vector2d(-11, -7), Math.toRadians(0))
                .splineTo(new Vector2d(25, -5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, -21), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    outtake.preset(Constants.Slides.superLow, Constants.Arm.placePos);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, true);
                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
                    telemetry.addData("did see one", optionalPose.isPresent());
                    telemetry.update();
                    cameras.setCameraSide(false);
                })
                .resetConstraints()
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 30, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 30  //acc
                )
                .lineToLinearHeading(new Pose2d(finalPlaceLocation.getX(), finalPlaceLocation.getY(), Math.toRadians(180)))
                .resetConstraints()
                .addTemporalMarkerOffset(-0.4, () -> {
                    if (goLeft && !hasTwo) {
                        autoServo.DropLeft();
                    } else if (!goLeft && !hasTwo) {
                        autoServo.DropRight();
                    } else if (goLeft && hasTwo) {
                        autoServo.DropRight();
                    } else if (!goLeft && hasTwo) {
                        autoServo.DropLeft();
                    }
                })
                .addTemporalMarkerOffset(0, () -> {
                    claw.setPower(Constants.Claw.outake);
                })
                .addTemporalMarkerOffset(0.1, () -> {
                    claw.setPower(0);
                    if (goLeft && !hasTwo) {
                        autoServo.DropRight();
                    } else if (!goLeft && !hasTwo) {
                        autoServo.DropLeft();
                    } else if (goLeft && hasTwo) {
                        autoServo.DropLeft();
                    } else if (!goLeft && hasTwo) {
                        autoServo.DropRight();
                    }
                })

                .addTemporalMarkerOffset(0.3, () -> {
                    claw.setPower(Constants.Claw.outake);
                })
                .addTemporalMarkerOffset(0.4, () -> {
                    outtake.presetSlides(Constants.Slides.low);
                    autoServo.upBoth();
                })
                .addTemporalMarkerOffset(1.5, () -> {
                    outtake.preset(Constants.Slides.intake, 0);
                    claw.setPower(0);
                })
                .waitSeconds(0.5)
                .setReversed(false)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 55, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 55  //acc
                )
                .splineToConstantHeading(new Vector2d(36, -10), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-30, -9, Math.toRadians(195)), Math.toRadians(180))
                .addTemporalMarkerOffset(0, () -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, false);
                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
                    telemetry.addData("did see two", optionalPose.isPresent());
                    telemetry.update();
                    cameras.setCameraSide(true);
                })

                //Back For Another One**************************************

                .resetConstraints()
                .lineToLinearHeading(new Pose2d(pickupSpecial2.getX(), pickupSpecial2.getY(), Math.toRadians(180)))

                .addTemporalMarkerOffset(-1, () -> {
                    intake.setIntakePower(Constants.Intake.intake, 0);
                    claw.setPower(Constants.Claw.intake);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down5);
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
                .addTemporalMarkerOffset(0.5, () -> {
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
                .splineTo(new Vector2d(-11, -9), Math.toRadians(0))
                .splineTo(new Vector2d(25, -10), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(37, -31), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    outtake.preset(Constants.Slides.med - 800, Constants.Arm.placePos);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, true);
                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
                    telemetry.addData("did see Three", optionalPose.isPresent());
                    telemetry.update();
                    cameras.kill();
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

    @Override
    public void loop() {
        stop();
    }
}
