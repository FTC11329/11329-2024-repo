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
import org.firstinspires.ftc.teamcode.subsystems.ClawSensor;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.SpecialIntake;
import org.firstinspires.ftc.teamcode.utility.BarcodePosition;

import java.util.Optional;

@Autonomous(name = "Blue Left 4 + 2 D C", group = "zCompetition")
@Config
public class BlueLeft4DC extends OpMode {
    static Pose2d startingPose = new Pose2d(17, 64, Math.toRadians(-90));
    static Vector2d placePositionOne = new Vector2d(49, 43.5);
    static Vector2d placePositionTwo = new Vector2d(49, 35);
    static Vector2d placePositionThree =  new Vector2d(49, 30);

    static Pose2d pickupSpecial = new Pose2d(-51,29, Math.toRadians(180));

    static double timeForPixelPlacement = 0.15;

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
        //1**************************************************************************
        placeSpikeMark1 = drivetrain.trajectorySequenceBuilder(startingPose)
                .addTemporalMarkerOffset(0, () -> {
                    outtake.createPresetThread(Constants.Slides.autoPurple, Constants.Arm.autoArmDrop, 3, Constants.Extendo.half, true);
                })
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .lineToLinearHeading(new Pose2d(35, 35, Math.toRadians(0)))
                .addTemporalMarkerOffset(0, () -> {
                    outtake.holdBackClaw(false);
                    outtake.extend(false);
                })
                .addTemporalMarkerOffset(0.25, () -> {
                    outtake.createPresetThread(Constants.Slides.superLow, Constants.Arm.placePos, 5, Constants.Extendo.half, true);
                })
                .lineToLinearHeading(new Pose2d(40, 36, Math.toRadians(-179)))
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
                .lineToLinearHeading(new Pose2d(27, 24.5, Math.toRadians(0)))
                .addTemporalMarkerOffset(0, () -> {
                    outtake.holdBackClaw(false);
                    outtake.extend(false);
                })
                .addTemporalMarkerOffset(0.5, () -> {
                    outtake.createPresetThread(Constants.Slides.superLow, Constants.Arm.placePos, 5, Constants.Extendo.half, true);
                })
                .lineToLinearHeading(new Pose2d(40, 36, Math.toRadians(-181)))
                .build();
        //3**************************************************************************
        placeSpikeMark3 = drivetrain.trajectorySequenceBuilder(startingPose)
                .addTemporalMarkerOffset(0, () -> {
                    outtake.createPresetThread(Constants.Slides.autoPurple, Constants.Arm.autoArmDrop, 3, Constants.Extendo.half, true);
                })
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .lineToLinearHeading(new Pose2d(20, 32, Math.toRadians(0)))
                .addTemporalMarkerOffset(-0.6, () -> {
                    outtake.extend(true);
                })
                .addTemporalMarkerOffset(0, () -> {
                    outtake.holdBackClaw(false);
                    outtake.extend(false);
                })
                .addTemporalMarkerOffset(0.5, () -> {
                    outtake.createPresetThread(Constants.Slides.superLow, Constants.Arm.placePos, 5, Constants.Extendo.half, true);
                })
                .lineToLinearHeading(new Pose2d(40, 36, Math.toRadians(180)))
                .build();
    }

    @Override
    public void init_loop() {
        boolean isBack = gamepad1.a;
        cameras.setCameraSide(gamepad1.a);

        BarcodePosition barcodePosition = distanceSensors.getDirectionBlue(false);
        telemetry.addData("Barcode Position", barcodePosition);
        telemetry.addData("FPS", cameras.switchingCamera.getFps());
        telemetry.addData("Is back", isBack);
        telemetry.update();
    }

    @Override
    public void start() {
        cameras.setCameraSideThreaded(true);
        BarcodePosition barcodePosition = distanceSensors.getDirectionBlue(false);

        drivetrain.getPoseEstimateOptical(startingPose);

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
        Vector2d finalPlaceLocation2 = placePositionOne.plus(new Vector2d(-3, -14));

        if (barcodePosition == BarcodePosition.One) {
            finalPlaceLocation  = placePositionOne;
        } else if (barcodePosition == BarcodePosition.Two) {
            finalPlaceLocation  = placePositionTwo;
        } else if (barcodePosition == BarcodePosition.Three) {
            finalPlaceLocation  = placePositionThree;
        } else return;


        drivetrain.followTrajectorySequenceAsync(drivetrain
                .trajectorySequenceBuilder(placeSpikeMark2.end())
                .resetConstraints()
                .setReversed(true)
                .addTemporalMarker(() -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, true);
                    optionalPose.ifPresent(pose2d -> drivetrain.getPoseEstimateOptical(pose2d));
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
                .addTemporalMarkerOffset(0.1, () -> {
                    outtake.holdClaw(false);
                    outtake.extend(false);
                })
                .addTemporalMarkerOffset(0.7, () -> {
                    outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false,false);
                })
                .waitSeconds(0.1)
                .setReversed(false)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 47, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 47  //acc
                )
                .splineToConstantHeading(new Vector2d(36, 8.75), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-30, 10.25, Math.toRadians(-220)), Math.toRadians(180))
                .addTemporalMarkerOffset(0, () -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, false);
                    optionalPose.ifPresent(pose2d -> drivetrain.getPoseEstimateOptical(pose2d));
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
                .forward(4)
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
                .splineTo(new Vector2d(-36, 14), Math.toRadians(0))
                .splineTo(new Vector2d(24, 14), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(finalPlaceLocation2.getX(), finalPlaceLocation2.getY() + 3), Math.toRadians(0))
                .addTemporalMarkerOffset(-1.75, () -> {
                    outtake.createPresetThread(Constants.Slides.superLow + 400, Constants.Arm.placePos, 5, Constants.Extendo.half, true);
                })
                .addTemporalMarkerOffset(0, () -> {
                    outtake.holdClaw(false);
                    outtake.extend(false);
                })
                .addTemporalMarkerOffset(0.5, () -> {
                    outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false,false);
                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(36, 6), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-30, 6, Math.toRadians(-225)), Math.toRadians(180))
                .addTemporalMarkerOffset(0, () -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, false);
                    optionalPose.ifPresent(pose2d -> drivetrain.getPoseEstimateOptical(pose2d));
                    telemetry.addData("did see four", optionalPose.isPresent());
                    telemetry.update();
                    cameras.kill();
                })
                //Back For Another One**************************************
                .lineToLinearHeading(pickupSpecial.plus(new Pose2d(-1.5,0)),
                        (displacement, pose, derivative, baseRobotVelocity) -> 32, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 32  //acc
                )
                .addTemporalMarkerOffset(-0.75, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down4);
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
                })
                .addTemporalMarkerOffset(2, () -> {
                    intake.setIntakePower(0, 0);
                    intake.setIntakeServoPower(0);
                })
                .splineTo(new Vector2d(-36, 14), Math.toRadians(0))
                .splineTo(new Vector2d(24, 12), Math.toRadians(0))
                .splineToConstantHeading(finalPlaceLocation2, Math.toRadians(0))
                .addTemporalMarkerOffset(-1.75, () -> {
                    outtake.createPresetThread(Constants.Slides.superLow + 400, Constants.Arm.placePos, 5, Constants.Extendo.half, true);
                })
                .addTemporalMarkerOffset(0, () -> {
                    outtake.holdClaw(false);
                    outtake.extend(false);
                })
                .addTemporalMarkerOffset(0.5, () -> {
                    outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false,false);
                })
                .waitSeconds(0.2)
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
    }
}