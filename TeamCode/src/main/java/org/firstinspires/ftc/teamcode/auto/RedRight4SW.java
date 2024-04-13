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

@Autonomous(name = "Red Right 4 + 2 S W", group = " Testing")
@Config
public class RedRight4SW extends OpMode {
    static Pose2d startingPose = new Pose2d(17, -64, Math.toRadians(90));
    static Vector2d placePositionOne = new Vector2d(52.5, -30.5);
    static Vector2d placePositionTwo = new Vector2d(52.5, -38.5);
    static Vector2d placePositionThree = new Vector2d(52.5, -40.5);

    static Pose2d pickupSpecial = new Pose2d(-54.5,-34, Math.toRadians(-195));

    static double timeForPixelPlacement = 0.15;

    TrajectorySequence placeSpikeMark1 = null;
    TrajectorySequence placeSpikeMark2 = null;
    TrajectorySequence placeSpikeMark3 = null;


    Claw claw;
    Intake intake;
    Outtake outtake;
    Cameras cameras;
    Drivetrain drivetrain;
    SpecialIntake specialIntake;
    DistanceSensors distanceSensors;

    public void init() {
        claw = new Claw(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        cameras = new Cameras(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        specialIntake = new SpecialIntake(hardwareMap);
        distanceSensors = new DistanceSensors(hardwareMap);
        //1**************************************************************************
        placeSpikeMark1 = drivetrain.trajectorySequenceBuilder(startingPose)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .lineToLinearHeading(new Pose2d(13, -36, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    outtake.presetArm(Constants.Arm.autoArmDrop);
                })
                .addTemporalMarkerOffset(timeForPixelPlacement, () -> {
                    outtake.presetArm(Constants.Arm.intakePos);
                })
                .waitSeconds(timeForPixelPlacement)
                .addTemporalMarkerOffset(0.5, () -> {
                    outtake.preset(Constants.Slides.superLow - 100, Constants.Arm.placePos);
                })
                .lineToLinearHeading(new Pose2d(40, -36, Math.toRadians(180)))
                .build();
        //2**************************************************************************
        placeSpikeMark2 = drivetrain.trajectorySequenceBuilder(startingPose)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .lineToLinearHeading(new Pose2d(27, -25, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    outtake.presetArm(Constants.Arm.autoArmDrop);
                })
                .addTemporalMarkerOffset(timeForPixelPlacement, () -> {
                    outtake.presetArm(Constants.Arm.intakePos);
                })
                .waitSeconds(timeForPixelPlacement)
                .addTemporalMarkerOffset(0.5, () -> {
                    outtake.preset(Constants.Slides.superLow - 100, Constants.Arm.placePos);
                })
                .lineToLinearHeading(new Pose2d(40, -36, Math.toRadians(181)))
                .build();
        //3**************************************************************************
        placeSpikeMark3 = drivetrain.trajectorySequenceBuilder(startingPose)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .lineToLinearHeading(new Pose2d(34, -30, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    outtake.presetArm(Constants.Arm.autoArmDrop);
                })
                .addTemporalMarkerOffset(timeForPixelPlacement, () -> {
                    outtake.presetArm(Constants.Arm.intakePos);
                })
                .waitSeconds(timeForPixelPlacement)
                .addTemporalMarkerOffset(0.5, () -> {
                    outtake.preset(Constants.Slides.superLow - 100, Constants.Arm.placePos);
                })
                .lineToLinearHeading(new Pose2d(40, -36, Math.toRadians(181)))
                .build();
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
        cameras.setCameraSide(true);
        BarcodePosition barcodePosition = distanceSensors.getDirectionRed(false);

        drivetrain.setPoseEstimate(startingPose);

        TrajectorySequence placeSpikeMarkActual = null;

        if (barcodePosition == BarcodePosition.One) {
            placeSpikeMarkActual = placeSpikeMark1;
        } else if (barcodePosition == BarcodePosition.Two) {
            placeSpikeMarkActual = placeSpikeMark2;
        } else if (barcodePosition == BarcodePosition.Three) {
            placeSpikeMarkActual = placeSpikeMark3;
        }

        drivetrain.followTrajectorySequence(placeSpikeMarkActual);

        Vector2d finalPlaceLocation = null;
        Vector2d finalPlaceLocation2 = new Vector2d(32, -54.5);

        if (barcodePosition == BarcodePosition.One) {
            finalPlaceLocation  = placePositionOne;
        } else if (barcodePosition == BarcodePosition.Two) {
            finalPlaceLocation  = placePositionTwo;
        } else if (barcodePosition == BarcodePosition.Three) {
            finalPlaceLocation  = placePositionThree;
        } else return;


        drivetrain.followTrajectorySequence(drivetrain
                .trajectorySequenceBuilder(placeSpikeMark2.end())
                .resetConstraints()
                .setReversed(true)
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
                .addTemporalMarkerOffset(-0.2, () -> {
                    claw.setPower(Constants.Claw.outake);
                })
                .addTemporalMarkerOffset(0.1, () -> {
                    outtake.presetSlides(Constants.Slides.low);
                })
                .addTemporalMarkerOffset(0.7, () -> {
                    outtake.preset(Constants.Slides.intake, Constants.Arm.intakePos);
                    claw.setPower(0);
                })
                .waitSeconds(0.2)
                .setReversed(false)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 45, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 45  //acc
                )
                .splineToConstantHeading(new Vector2d(16, -62), Math.toRadians(180))
                .splineTo(new Vector2d(-25, -61), Math.toRadians(180))
                .splineTo(new Vector2d(-43, -50), Math.toRadians(160))
                .addTemporalMarkerOffset(0, () -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, false);
                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
                    telemetry.addData("did see two", optionalPose.isPresent());
                    telemetry.update();
                    cameras.setCameraSide(true);
//                    cameras.kill();
                })

                //Back For Another One**************************************
                .lineToLinearHeading(pickupSpecial,
                        (displacement, pose, derivative, baseRobotVelocity) -> 30, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 30  //acc
                )
                .addTemporalMarkerOffset(-0.5, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                })
                .addTemporalMarkerOffset(-0.2, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down5);
                    intake.setIntakePower(Constants.Intake.intake, 0);
                    claw.setPower(Constants.Claw.intake);
                    outtake.presetArm(Constants.Arm.intakePos);
                })
                .addTemporalMarkerOffset(0, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down4);
                })
                .addTemporalMarkerOffset(0.5, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.up);
                })
                .waitSeconds(0.5)
                .forward(2.5)
                .setReversed(true)
                .addTemporalMarkerOffset(2, () -> {
                    intake.setIntakePower(Constants.Intake.outake, 0);
                    claw.setPower(0);
                })
                .addTemporalMarkerOffset(3, () -> {
                    intake.setIntakePower(0, 0);
                })
                .splineTo(new Vector2d(-35, -54.5), Math.toRadians(0))
                .splineTo(new Vector2d(finalPlaceLocation2.getX(), finalPlaceLocation2.getY()), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    claw.setPower(Constants.Claw.outake);
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, true);
                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
                    telemetry.addData("did see Three", optionalPose.isPresent());
                    telemetry.update();
                    cameras.setCameraSide(false);
                })
                .addTemporalMarkerOffset(-1, () -> {
                    outtake.preset(Constants.Slides.low, Constants.Arm.armMax);
                })
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 30, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 30  //acc
                )
                .addTemporalMarkerOffset(0.1, () -> {
                    outtake.preset(Constants.Slides.intake, Constants.Arm.intakePos);
                    claw.setPower(0);
                })
                .resetConstraints()
                .setReversed(false)
                .splineTo(new Vector2d(-25, -61), Math.toRadians(180))
                .splineTo(new Vector2d(-43, -50), Math.toRadians(160))
                .addTemporalMarkerOffset(0, () -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, false);
                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
                    telemetry.addData("did see four", optionalPose.isPresent());
                    telemetry.update();
                    cameras.kill();
                })
                //Back For Another One**************************************
                .lineToLinearHeading(pickupSpecial,
                        (displacement, pose, derivative, baseRobotVelocity) -> 30, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 30  //acc
                )
                .addTemporalMarkerOffset(-0.5, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down4);
                })
                .addTemporalMarkerOffset(-0.25, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down3);
                    intake.setIntakePower(Constants.Intake.intake, 0);
                    claw.setPower(Constants.Claw.intake);
                    outtake.presetArm(Constants.Arm.intakePos);
                })
                .addTemporalMarkerOffset(0, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down2);
                })
                .addTemporalMarkerOffset(0.5, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.up);
                })
                .waitSeconds(0.5)
                .forward(2.5)
                .setReversed(true)
                .addTemporalMarkerOffset(2, () -> {
                    intake.setIntakePower(Constants.Intake.outake, 0);
                    claw.setPower(0);
                })
                .addTemporalMarkerOffset(3, () -> {
                    intake.setIntakePower(0, 0);
                })
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 35, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 35  //acc
                )
                .splineTo(new Vector2d(-35, -54.5), Math.toRadians(0))
                .splineTo(new Vector2d(19, -55.5), Math.toRadians(0))
                .splineTo(new Vector2d(finalPlaceLocation2.getX(), finalPlaceLocation2.getY()), Math.toRadians(0))
                .addTemporalMarkerOffset(-1, () -> {
                    outtake.presetArm(Constants.Arm.fixPos);
                })
                .addTemporalMarkerOffset(-0.25, () -> {
                    claw.setPower(Constants.Claw.outake);
                })
                .forward(10)
                .addTemporalMarkerOffset(-0.5, () -> {
                    claw.setPower(0);
                    outtake.presetArm(Constants.Arm.intakePos);
                })
                //actual
                .lineToLinearHeading(new Pose2d(49,-60, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(57,-60, Math.toRadians(180)))
                //re-lineup
//                .lineToLinearHeading(new Pose2d(15, -55, Math.toRadians(90)))
                .build());
    }

    @Override
    public void loop() {
        stop();
    }
}