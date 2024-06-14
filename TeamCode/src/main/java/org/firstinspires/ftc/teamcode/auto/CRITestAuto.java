package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
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

@Autonomous(name = "Red Center CRI", group = " Testing")
@Config
public class CRITestAuto extends OpMode {
    boolean whiteLeft;
    static Pose2d startingPose = new Pose2d(-8, -63, Math.toRadians(90));
    static Vector2d placePositionOne = new Vector2d(74, -30);
    static Vector2d placePositionTwo = new Vector2d(74, -34);
    static Vector2d placePositionThree = new Vector2d(74, -34);

    static Pose2d pickupSpecial = new Pose2d(-17.5, -12.5, Math.toRadians(135));
    static Pose2d pickupSpecial2 = new Pose2d(-17.5,-12.5, Math.toRadians(135));

    TrajectorySequence placeSpikeMark1 = null;
    TrajectorySequence placeSpikeMark2 = null;
    TrajectorySequence placeSpikeMark3 = null;

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
        drivetrain = new Drivetrain(hardwareMap);
        specialIntake = new SpecialIntake(hardwareMap);
        distanceSensors = new DistanceSensors(hardwareMap);
        claw.setBackHold(true);
        //1**************************************************************************
        placeSpikeMark1 = drivetrain.trajectorySequenceBuilder(startingPose)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .addTemporalMarker(() -> {
                    outtake.createPresetThread(150, Constants.Arm.autoArmDrop, 3, Constants.Extendo.auto, true);
                })
                .lineToLinearHeading(new Pose2d(-15, -31, Math.toRadians(0)))
                .waitSeconds(0.1)
                .addTemporalMarkerOffset(0.1, () -> {
                    outtake.holdClaw(false);
                })
                .addTemporalMarkerOffset(0.15, () -> {
                    outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false,false);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                })
                .waitSeconds(0.15)
                .lineToLinearHeading(new Pose2d(-12,-30, 90))
                .lineToLinearHeading(pickupSpecial.plus(new Pose2d(3,0, 0)))
                .build();
        //2**************************************************************************
        placeSpikeMark2 = drivetrain.trajectorySequenceBuilder(startingPose)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .addTemporalMarker(() -> {
                    outtake.createPresetThread(150, Constants.Arm.autoArmDrop, 3, Constants.Extendo.auto, true);
                })
                .lineToLinearHeading(new Pose2d(-12, -21.5, Math.toRadians(90)))
                .waitSeconds(0.1)
                .addTemporalMarkerOffset(0.1, () -> {
                    outtake.holdClaw(false);
                })
                .addTemporalMarkerOffset(0.15, () -> {
                    outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false,false);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                })
                .waitSeconds(0.15)
                .lineToLinearHeading(pickupSpecial.plus(new Pose2d(3,0, 0)))
                .build();
        //3**************************************************************************
        placeSpikeMark3 = drivetrain.trajectorySequenceBuilder(startingPose)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .addTemporalMarker(() -> {
                    outtake.createPresetThread(150, Constants.Arm.autoArmDrop, 3, Constants.Extendo.auto, true);
                })
                .lineToLinearHeading(new Pose2d(-11, -35, Math.toRadians(180)))
                .waitSeconds(0.1)
                .addTemporalMarkerOffset(0.1, () -> {
                    outtake.holdClaw(false);
                })
                .addTemporalMarkerOffset(0.15, () -> {
                    outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false,false);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                })
                .waitSeconds(0.15)
                .turn(Math.toRadians(-90))
                .splineToLinearHeading(pickupSpecial.plus(new Pose2d(5,0, 0)), Math.toRadians(90))
                .build();
    }

    @Override
    public void init_loop() {
        boolean isBack = gamepad1.a;
        cameras.setCameraSide(isBack);

        BarcodePosition barcodePosition = distanceSensors.getDirectionRed(false);
        telemetry.addData("Barcode Position", barcodePosition);
        telemetry.addData("FPS", cameras.switchingCamera.getFps());
        telemetry.addData("Is back", isBack);
        telemetry.addData("Front color", clawSensor.getFrontColor());
        telemetry.addData("Back Color" , clawSensor.getBackColor());
        telemetry.update();
    }

    @Override
    public void start() {
        cameras.setCameraSideThreaded(true);
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
        Vector2d finalPlaceLocation2 = null;

        if (barcodePosition == BarcodePosition.One) {
            finalPlaceLocation  = placePositionOne;
            finalPlaceLocation2 = new Vector2d(60, -10);
            whiteLeft = false;

        } else if (barcodePosition == BarcodePosition.Two) {
            finalPlaceLocation  = placePositionTwo;
            finalPlaceLocation2 = new Vector2d(60, -10);
            whiteLeft = true;

        } else {
            finalPlaceLocation  = placePositionThree;
            finalPlaceLocation2 = new Vector2d(60, -10);
            whiteLeft = true;
        }


        drivetrain.followTrajectorySequenceAsync(drivetrain
                .trajectorySequenceBuilder(placeSpikeMark2.end())
                .resetConstraints()
                .setReversed(true)
                .lineToLinearHeading(pickupSpecial)
                //1st pickup start
                .addTemporalMarkerOffset(-0.2, () -> {
                    intake.setIntakePower(Constants.Intake.intake, 0);
                    intake.setIntakeServoPower(0.75);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down5);
                })
                .addTemporalMarkerOffset(0.1, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.up);
                })
                //1st pickup end
                .waitSeconds(0.1)
                .forward(1.73)
                .setReversed(true)
                .addTemporalMarkerOffset(2, () -> {
                    intake.setIntakePower(Constants.Intake.outake, 0);
                })
                .addTemporalMarkerOffset(3, () -> {
                    intake.setIntakePower(0, 0);
                })
                .splineTo(new Vector2d(0, -12), Math.toRadians(0))
                .splineTo(new Vector2d(40, -12), Math.toRadians(0))
                .splineTo(finalPlaceLocation, Math.toRadians(0))
                .addTemporalMarkerOffset(-1, () -> {
                    intake.setIntakePower(0, 0);
                    intake.setIntakeServoPower(0);
                    outtake.createPresetThread(Constants.Slides.superLow, Constants.Arm.placePos, 1, Constants.Extendo.half, true);
                })
//                .addTemporalMarkerOffset(-0.9, () -> {
//                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, true);
//                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
//                    telemetry.addData("did see one", optionalPose.isPresent());
//                    telemetry.update();
//                })
                //1st place
                .addTemporalMarkerOffset(0.2, () -> {
                    outtake.holdClaw(false);
                    outtake.extend(false);
                })
                .waitSeconds(0.1)
                .setReversed(false)
                .addTemporalMarkerOffset(0.2, () -> {
                    intake.setIntakePower(0, 0);
                    intake.setIntakeServoPower(0);
                    outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false, false);
                })
                .splineTo(new Vector2d(40, -12), Math.toRadians(180))
                .splineTo(new Vector2d(0, -12), Math.toRadians(180))
                .splineToSplineHeading(pickupSpecial2, Math.toRadians(180))
                //2nd pickup start
                .addTemporalMarkerOffset(-0.5, () -> {
                    intake.setIntakePower(Constants.Intake.intake, 0);
                    intake.setIntakeServoPower(0.75);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down4);
                })
                .addTemporalMarkerOffset(0, () -> {
                    intake.setIntakePower(Constants.Intake.intake, 0);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down3);
                })
                //2nd pickup done
                .waitSeconds(0.1)
                .forward(1.73)
                .setReversed(true)
                .addTemporalMarkerOffset(2, () -> {
                    intake.setIntakePower(Constants.Intake.outake, 0);
                })
                .addTemporalMarkerOffset(3, () -> {
                    intake.setIntakePower(0, 0);
                })
                .splineTo(new Vector2d(0, -12), Math.toRadians(0))
                .splineTo(new Vector2d(24, -12), Math.toRadians(0))
                .splineTo(finalPlaceLocation2, Math.toRadians(0))
                .addTemporalMarkerOffset(-1, () -> {
                    intake.setIntakePower(0, 0);
                    intake.setIntakeServoPower(0);
                    outtake.createPresetThread(Constants.Slides.superLow, Constants.Arm.placePos, 1, Constants.Extendo.half, true);
                })
//                .addTemporalMarkerOffset(-0.9, () -> {
//                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, true);
//                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
//                    telemetry.addData("did see one", optionalPose.isPresent());
//                    telemetry.update();
//                })
                //2nd place
                .addTemporalMarkerOffset(0.2, () -> {
                    outtake.holdClaw(false);
                    outtake.extend(false);
                })
                .waitSeconds(0.1)
                .strafeRight(6)
                .addTemporalMarkerOffset(-0.5, () -> {
                    intake.setIntakePower(0, 0);
                    intake.setIntakeServoPower(0);
                    outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false, false);
                })
                .waitSeconds(10)
                .build());
    }

    @Override
    public void loop() {
        drivetrain.update();
        if (clawSensor.autoSense()) {
            intake.setIntakePower(Constants.Intake.outake, 0);
        }
    }
}
