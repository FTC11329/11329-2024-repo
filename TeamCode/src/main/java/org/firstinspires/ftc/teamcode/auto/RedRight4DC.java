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

@Autonomous(name = "Red Right 4 + 2 D C", group = " Competition")
@Config
public class RedRight4DC extends OpMode {
    static Pose2d startingPose = new Pose2d(17, -64, Math.toRadians(90));
    static Vector2d placePositionOne = new Vector2d(49, -25);
    static Vector2d placePositionTwo = new Vector2d(49, -32);
    static Vector2d placePositionThree = new Vector2d(49, -40.5);

    static Pose2d pickupSpecial = new Pose2d(-58,-17.5, Math.toRadians(180));

    TrajectorySequence placeSpikeMark1 = null;
    TrajectorySequence placeSpikeMark2 = null;
    TrajectorySequence placeSpikeMark3 = null;


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
        //1**************************************************************************
        placeSpikeMark1 = drivetrain.trajectorySequenceBuilder(startingPose)
                .addTemporalMarkerOffset(0, () -> {
                    outtake.createPresetThread(Constants.Slides.autoPurple, Constants.Arm.autoArmDrop, 3, Constants.Extendo.half, true);
                })
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .lineToLinearHeading(new Pose2d(20, -34, Math.toRadians(0)))
                .addTemporalMarkerOffset(-0.6, () -> {
                    outtake.extend(true);
                })
                .addTemporalMarkerOffset(0, () -> {
                    outtake.holdBackClaw(false);
                })
                .addTemporalMarkerOffset(0.5, () -> {
                    outtake.createPresetThread(Constants.Slides.superLow, Constants.Arm.placePos, 5, Constants.Extendo.half + 0.05, true);
                })
                .lineToLinearHeading(new Pose2d(40, -36, Math.toRadians(180)))
                .build();
        //2**************************************************************************
        placeSpikeMark2 = drivetrain.trajectorySequenceBuilder(startingPose)
                .addTemporalMarkerOffset(0, () -> {
                    outtake.createPresetThread(Constants.Slides.autoPurple, Constants.Arm.autoArmDrop, 3, Constants.Extendo.auto, true);
                })
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .lineToLinearHeading(new Pose2d(28.5, -24.25, Math.toRadians(0)))
                .addTemporalMarkerOffset(0, () -> {
                    outtake.holdBackClaw(false);
                })
                .addTemporalMarkerOffset(0.25, () -> {
                    outtake.presetArm(Constants.Arm.placePos);
                })
                .addTemporalMarkerOffset(0.5, () -> {
                    outtake.createPresetThread(Constants.Slides.superLow, Constants.Arm.placePos, 5, Constants.Extendo.half + 0.05, true);
                })
                .lineToLinearHeading(new Pose2d(40, -36, Math.toRadians(181)))
                .build();
        //3**************************************************************************
        placeSpikeMark3 = drivetrain.trajectorySequenceBuilder(startingPose)
                .addTemporalMarkerOffset(0, () -> {
                    outtake.createPresetThread(Constants.Slides.autoPurple, Constants.Arm.autoArmDrop, 3, Constants.Extendo.auto, true);
                })
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .lineToLinearHeading(new Pose2d(35, -30, Math.toRadians(0)))
                .addTemporalMarkerOffset(0, () -> {
                    outtake.holdBackClaw(false);
                })
                .addTemporalMarkerOffset(0.25, () -> {
                    outtake.createPresetThread(Constants.Slides.superLow, Constants.Arm.placePos, 5, Constants.Extendo.half + 0.05, true);
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

        Vector2d finalPlaceLocation;
        Vector2d finalPlaceLocation2;
        Vector2d finalPlaceLocation3;
        if (barcodePosition == BarcodePosition.One) {
            finalPlaceLocation  = placePositionOne;
            finalPlaceLocation2 = placePositionOne.plus(new Vector2d(0, -6));
            finalPlaceLocation3 = placePositionOne.plus(new Vector2d(1, -12));

        } else if (barcodePosition == BarcodePosition.Two) {
            finalPlaceLocation  = placePositionTwo;
            finalPlaceLocation2 = placePositionOne.plus(new Vector2d(0, -4));
            finalPlaceLocation3 = placePositionOne.plus(new Vector2d(1, -11));

        } else if (barcodePosition == BarcodePosition.Three) {
            finalPlaceLocation  = placePositionThree;
            finalPlaceLocation2 = placePositionOne.plus(new Vector2d(0, -2));
            finalPlaceLocation3 = placePositionOne.plus(new Vector2d(0, -2));

        } else return;


        drivetrain.followTrajectorySequenceAsync(drivetrain
                .trajectorySequenceBuilder(placeSpikeMark2.end())
                .resetConstraints()
                .setReversed(true)
                .addTemporalMarker(() -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, true);
                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
                    telemetry.addData("did see one", optionalPose.isPresent());
                    telemetry.update();
                    cameras.setCameraSideThreaded(false);
                })
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 30, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 30  //acc
                )
                .lineToLinearHeading(new Pose2d(finalPlaceLocation.getX(), finalPlaceLocation.getY(), Math.toRadians(180)))
                .resetConstraints()
                .addTemporalMarkerOffset(0, () -> {
                    outtake.holdClaw(false);
                    outtake.extend(false);
                })
                .addTemporalMarkerOffset(0.3, () -> {
                    outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false,false);
                })
                .setReversed(false)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 45, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 45  //acc
                )
                .splineToConstantHeading(new Vector2d(36, -9), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-27, -6, Math.toRadians(190)), Math.toRadians(180))
                .waitSeconds(0.1)
                .addTemporalMarkerOffset(0, () -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, false);
                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
                    telemetry.addData("did see two", optionalPose.isPresent());
                    telemetry.update();
                })
                //Back For Another One**************************************
                .lineToLinearHeading(pickupSpecial,
                        (displacement, pose, derivative, baseRobotVelocity) -> 32, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 32  //acc
                )
                .addTemporalMarkerOffset(-0.75, () -> {
                    outtake.presetSlides(Constants.Slides.whileIntaking);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                    clawSensor.setRunInAuto(true);
                })
                .addTemporalMarkerOffset(-0.5, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down4);
                    intake.setIntakePower(Constants.Intake.intake, 0);
                    intake.setIntakeServoPower(Constants.Intake.intakeServoIntake);
                })
                .addTemporalMarkerOffset(0, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.up);
                })
                .waitSeconds(0.1)
                .forward(2)
                .setReversed(true)
                .addTemporalMarkerOffset(0.75, () -> {
                    clawSensor.setRunInAuto(false);
                    outtake.presetSlides(-50);
                    intake.setIntakePower(Constants.Intake.outake, 0);
                })
                .addTemporalMarkerOffset(2, () -> {
                    intake.setIntakePower(0, 0);
                    intake.setIntakeServoPower(0);
                    outtake.holdClaw(true);
                })
                .splineTo(new Vector2d(-36, -10), Math.toRadians(0))
                .splineTo(new Vector2d(23, -10), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(finalPlaceLocation2.getX(), finalPlaceLocation2.getY()), Math.toRadians(0))
                .addTemporalMarkerOffset(-1.75, () -> {
                    outtake.createPresetThread(Constants.Slides.superLow + 400, Constants.Arm.placePos, 5, Constants.Extendo.half, true);
                })
                .addTemporalMarkerOffset(-0.1, () -> {
                    outtake.holdClaw(false);
                    outtake.extend(false);
                })
                .addTemporalMarkerOffset(0.2, () -> {
                    outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false,false);
                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(36, -9), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-30, -6, Math.toRadians(195)), Math.toRadians(180))
                .addTemporalMarkerOffset(0, () -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, false);
                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
                    telemetry.addData("did see four", optionalPose.isPresent());
                    telemetry.update();
                    cameras.kill();
                })
                //Back For Another One**************************************
                .lineToLinearHeading(pickupSpecial,
                        (displacement, pose, derivative, baseRobotVelocity) -> 32, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 32  //acc
                )
                .addTemporalMarkerOffset(-0.75, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down4);
                    outtake.presetSlides(Constants.Slides.whileIntaking);
                    clawSensor.setRunInAuto(true);
                })
                .addTemporalMarkerOffset(-0.5, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down2);
                    intake.setIntakePower(Constants.Intake.intake, 0);
                    intake.setIntakeServoPower(Constants.Intake.intakeServoIntake);
                })
                .addTemporalMarkerOffset(0, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.up);
                })
                .waitSeconds(0.1)
                .forward(5.5)
                .setReversed(true)
                .addTemporalMarkerOffset(0.75, () -> {
                    clawSensor.setRunInAuto(false);
                    intake.setIntakePower(Constants.Intake.outake, 0);
                    outtake.presetSlides(-50);
                })
                .addTemporalMarkerOffset(2, () -> {
                    intake.setIntakePower(0, 0);
                    intake.setIntakeServoPower(0);
                })
                .splineTo(new Vector2d(-36,-9), Math.toRadians(0))
                .splineTo(new Vector2d(23, -9), Math.toRadians(0))
                .splineToConstantHeading(finalPlaceLocation3, Math.toRadians(0))
                .addTemporalMarkerOffset(-1.75, () -> {
                    outtake.createPresetThread(Constants.Slides.superLow + 400, Constants.Arm.placePos, 5, Constants.Extendo.half, true);
                })
                .addTemporalMarkerOffset(0, () -> {
                    outtake.holdClaw(false);
                    outtake.extend(false);
                })
                .addTemporalMarkerOffset(0.3, () -> {
                    outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false,false);
                })
                .forward(7)
                .waitSeconds(1)
                .build());
    }

    @Override
    public void loop() {
        drivetrain.update();
        outtake.periodic();
        if (clawSensor.autoSense()) {
            intake.setIntakePower(Constants.Intake.outake, 0);
        }
        telemetry.addData("slides target", outtake.getSlideTargetPosition());
        telemetry.addData("slides actual", outtake.getSlidePosition());
    }
}