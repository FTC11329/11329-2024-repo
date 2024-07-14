package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystems.Cameras;
import org.firstinspires.ftc.teamcode.subsystems.ClawSensor;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.SpecialIntake;
import org.firstinspires.ftc.teamcode.utility.BarcodePosition;

import java.util.Optional;

@Autonomous(name = "A Blue Right Center CRI", group = " Testing") //yellow
@Config
public class ACRIBlueRightCenter extends OpMode {
    int wristPos;
    static Pose2d startingPose = new Pose2d(-64.5, 63, Math.toRadians(-90));
    static Vector2d finalPlacePos;
    static Vector2d finalPlacePos2 = new Vector2d(71.5, -2);

    static Pose2d pickupSpecial = new Pose2d(-74, 14.5, Math.toRadians(-90));

    static Vector2d prePickup = new Vector2d(0, 0);
    static Pose2d pickupSpecial2 = new Pose2d(-11,0, Math.toRadians(-135)); //normal ***********************************************
    static double yComingBack = -5;
//    static Vector2d prePickup = new Vector2d(-24, 18);
//    static Pose2d pickupSpecial2 = new Pose2d(-34, 18, Math.toRadians(-135)); //far    ***********************************************
//    static double yComingBack = 24;


    TrajectorySequenceBuilder placeSpikeMark1 = null;
    TrajectorySequenceBuilder placeSpikeMark2 = null;
    TrajectorySequenceBuilder placeSpikeMark3 = null;

    TrajectorySequenceBuilder restOfIt1 = null;
    TrajectorySequenceBuilder restOfIt2 = null;
    TrajectorySequenceBuilder restOfIt3 = null;
    TrajectorySequenceBuilder restOfIt4 = null;

    Optional<Pose2d> optionalPose;

    Intake intake;
    Lights lights;
    Outtake outtake;
    Cameras cameras;
    ClawSensor clawSensor;
    Drivetrain drivetrain;
    SpecialIntake specialIntake;
    DistanceSensors distanceSensors;

    ConstantCRIPathsBlue constantCRIPaths;
    ConstantCRIPathsBlue.PlacePurplePaths placePurplePaths;
    ConstantCRIPathsBlue.PickupWhitePixelStack pickupWhitePixelStack;
    ConstantCRIPathsBlue.PlaceOnBackDrop placeOnBackDrop;


    public void init() {
        lights = new Lights(hardwareMap);
        lights.setDumbLed(1);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        cameras = new Cameras(hardwareMap);
        clawSensor = new ClawSensor(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        specialIntake = new SpecialIntake(hardwareMap);
        distanceSensors = new DistanceSensors(hardwareMap);
        outtake.holdBackClaw(true);

        constantCRIPaths = new ConstantCRIPathsBlue(telemetry, intake, outtake, cameras, clawSensor, drivetrain, specialIntake, new Pose2d(0,0, Math.toRadians(0)), new Pose2d(0,0, Math.toRadians(0)), new Vector2d(0,0));
        placePurplePaths = constantCRIPaths.placePurplePathsBlue;
        pickupWhitePixelStack = constantCRIPaths.pickupWhitePixelStack;
        placeOnBackDrop = constantCRIPaths.placeOnBackDrop;

        //1**************************************************************************
        placeSpikeMark1 = drivetrain.trajectorySequenceBuilder(startingPose);
        placePurplePaths.RightPlacePos1Center.run(placeSpikeMark1);
        //2**************************************************************************
        placeSpikeMark2 = drivetrain.trajectorySequenceBuilder(startingPose);
        placePurplePaths.RightPlacePos2Center.run(placeSpikeMark2);
        //3**************************************************************************
        placeSpikeMark3 = drivetrain.trajectorySequenceBuilder(startingPose);
        placePurplePaths.RightPlacePos3Center.run(placeSpikeMark3);
    }

    @Override
    public void init_loop() {
        boolean isBack = gamepad1.a;
        cameras.setCameraSide(isBack);

        BarcodePosition barcodePosition = distanceSensors.getDirectionBlue(true);
        telemetry.addData("Barcode Position", barcodePosition);
        telemetry.addData("FPS", cameras.switchingCamera.getFps());
        telemetry.addData("Is back", isBack);
        telemetry.addData("Front color", clawSensor.getFrontColor());
        telemetry.addData("Back Color" , clawSensor.getBackColor());
        telemetry.update();
        if (cameras.switchingCamera.getFps() > 10) {
            lights.setDumbLed(0);
        }
    }

    @Override
    public void start() {
        cameras.setCameraSideThreaded(true);
        BarcodePosition barcodePosition = distanceSensors.getDirectionBlue(true);

        drivetrain.setPoseEstimate(startingPose);

        TrajectorySequence placeSpikeMarkActual;

        if (barcodePosition == BarcodePosition.One) {
            placeSpikeMarkActual = placeSpikeMark1.build();
        } else if (barcodePosition == BarcodePosition.Two) {
            placeSpikeMarkActual = placeSpikeMark2.build();
        } else {
            placeSpikeMarkActual = placeSpikeMark3.build();
        }

        if (barcodePosition == BarcodePosition.One) {
            finalPlacePos = new Vector2d(72, 39);
            wristPos = 5;

        } else if (barcodePosition == BarcodePosition.Two) {
            finalPlacePos = new Vector2d(72, 34);
            wristPos = 1;

        } else {//if barcodePosition == BarcodePosition.Three
            finalPlacePos = new Vector2d(72, 31);
            wristPos = 1;
        }

        drivetrain.followTrajectorySequence(placeSpikeMarkActual);
        restOfIt1 = drivetrain.trajectorySequenceBuilder(placeSpikeMarkActual.end());
        restOfIt1
                .resetConstraints()
                .setReversed(true)
                .lineToLinearHeading(pickupSpecial)
                //1st pickup start
                .addTemporalMarkerOffset(-0.2, () -> {
                    clawSensor.setRunInAuto(true);
                    outtake.presetSlides(Constants.Slides.whileIntaking);
                    intake.setIntakePower(Constants.Intake.intake, 0);
                    intake.setIntakeServoPower(Constants.Intake.intakeServoIntake);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down5);
                })
                .addTemporalMarkerOffset(0.1, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.up);
                })
                //1st pickup end
                .waitSeconds(0.1)
                .forward(2.75)
                .setReversed(true)
                .addTemporalMarkerOffset(1.5, () -> {
                    intake.setIntakePower(Constants.Intake.outake, 0);
                    outtake.presetSlides(0);
                    clawSensor.setRunInAuto(false);
                    outtake.presetSlides(-5);
                    outtake.holdClaw(true);
                })
                .addTemporalMarkerOffset(3, () -> {
                    intake.setIntakePower(0, 0);
                    intake.setIntakeServoPower(0);
                })

                .lineToLinearHeading(new Pose2d(-65, 10, Math.toRadians(180)))
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 55, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 55  //acc
                )
                .splineTo(new Vector2d(-12, 4), Math.toRadians(0))
                .splineTo(new Vector2d(40, 4), Math.toRadians(0))
                .waitSeconds(0.01)
                .addTemporalMarkerOffset(0, () -> {
                    double distance = 30.0;
                    while (distance > 15.0) {
                        optionalPose = cameras.getRunnerPoseEstimate(0, false);
                        boolean present = optionalPose.isPresent();
                        if (present) {
                            distance = Math.sqrt(Math.pow(drivetrain.getPoseEstimate().getX() - optionalPose.get().getX(), 2) + Math.pow(drivetrain.getPoseEstimate().getY() - optionalPose.get().getY(), 2));
                            if (distance < 15.0) {
                                drivetrain.setPoseEstimate(optionalPose.get());
                            }
                        }
                        telemetry.addData("distance = ", distance);
                        telemetry.addData("did see one", optionalPose.isPresent());
                        telemetry.update();
                    }
                });
        drivetrain.followTrajectorySequence(restOfIt1.build());
        restOfIt2 = drivetrain.trajectorySequenceBuilder(optionalPose.get());
        restOfIt2
                .resetConstraints()
                .splineToConstantHeading(finalPlacePos, Math.toRadians(0))
                .addTemporalMarkerOffset(-2, () -> {
                    intake.setIntakePower(0, 0);
                    intake.setIntakeServoPower(0);
                    outtake.createPresetThread(Constants.Slides.superLow + 200, Constants.Arm.placePos, wristPos, Constants.Extendo.extended, true);
                })
                .addTemporalMarkerOffset(0, () -> {
                    outtake.holdClaw(false);
                    outtake.extend(false);
                })
                .addTemporalMarkerOffset(0.5, () -> {
                    intake.setIntakePower(0, 0);
                    intake.setIntakeServoPower(0);
                    outtake.createPresetThread(5, Constants.Arm.intakePos, 3, false, false);
                })
                .setReversed(false)
                .waitSeconds(0.1)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 30, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 30  //acc
                )
                .splineToConstantHeading(new Vector2d(50, 0), Math.toRadians(180))
                .addTemporalMarkerOffset(0, () -> {
                    double distance = 30.0;
                    while (distance > 15.0) {
                        optionalPose = cameras.getRunnerPoseEstimate(0, false);
                        boolean present = optionalPose.isPresent();
                        if (present) {
                            distance = Math.sqrt(Math.pow(drivetrain.getPoseEstimate().getX() - optionalPose.get().getX(), 2) + Math.pow(drivetrain.getPoseEstimate().getY() - optionalPose.get().getY(), 2));
                            if (distance < 15.0) {
                                drivetrain.setPoseEstimate(optionalPose.get());
                            }
                        }
                        telemetry.addData("distance = ", distance);
                        telemetry.addData("did see one", optionalPose.isPresent());
                        telemetry.update();
                    }
                })
                .waitSeconds(0.1);
        drivetrain.followTrajectorySequence(restOfIt2.build());
        restOfIt3 = drivetrain.trajectorySequenceBuilder(optionalPose.get());
        restOfIt3
                .waitSeconds(0.25)
                .splineToConstantHeading(prePickup, Math.toRadians(180))
                .splineToSplineHeading(pickupSpecial2, Math.toRadians(180))
                //stack intake
                .addTemporalMarkerOffset(-0.5, () -> {
                    clawSensor.setRunInAuto(true);
                    outtake.presetSlides(Constants.Slides.whileIntaking);
                    intake.setIntakePower(Constants.Intake.intake, 0);
                    intake.setIntakeServoPower(Constants.Intake.intakeServoIntake);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down4);
                })
                .addTemporalMarkerOffset(0.05 , () -> {
                    intake.setIntakePower(Constants.Intake.intake, 0);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down2);

                })
                .waitSeconds(0.1)
                .forward(2.75)
                .addTemporalMarkerOffset(0.5 , () -> {
                    outtake.presetSlides(-50);
                })
                .setReversed(true)
                .addTemporalMarkerOffset(1.5, () -> {
                    intake.setIntakePower(Constants.Intake.outake, 0);
                    clawSensor.setRunInAuto(false);
                    outtake.holdClaw(true);
                })
                .addTemporalMarkerOffset(3, () -> {
                    intake.setIntakePower(0, 0);
                })
                .lineToLinearHeading(new Pose2d(0, yComingBack, Math.toRadians(180)))
                .lineTo(new Vector2d(40, yComingBack))
                .addTemporalMarkerOffset(0, () -> {
                    double distance = 30.0;
                    while (distance > 20.0) {
                        optionalPose = cameras.getRunnerPoseEstimate(0, false);
                        boolean present = optionalPose.isPresent();
                        if (present) {
                            distance = Math.sqrt(Math.pow(drivetrain.getPoseEstimate().getX() - optionalPose.get().getX(), 2) + Math.pow(drivetrain.getPoseEstimate().getY() - optionalPose.get().getY(), 2));
                            if (distance < 20.0) {
                                drivetrain.setPoseEstimate(optionalPose.get());
                            }
                        }
                        telemetry.addData("distance = ", distance);
                        telemetry.addData("did see one", optionalPose.isPresent());
                        telemetry.update();
                    }
                })
                .waitSeconds(0.01);
        drivetrain.followTrajectorySequence(restOfIt3.build());
        restOfIt4 = drivetrain.trajectorySequenceBuilder(optionalPose.get());
        restOfIt4
                .splineToConstantHeading(finalPlacePos2, Math.toRadians(0))
                .addTemporalMarkerOffset(-2.4, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.up);
                    intake.setIntakePower(0, 0);
                    intake.setIntakeServoPower(0);
                    outtake.createPresetThread(Constants.Slides.high, Constants.Arm.placePos, 1, Constants.Extendo.half, true);
                })
                .waitSeconds(0.2)
                .addTemporalMarkerOffset(0.2, () -> {
                    outtake.holdClaw(false);
                    outtake.extend(false);
                })
                .waitSeconds(0.1)
                .strafeRight(13)
                .addTemporalMarkerOffset(-0.5, () -> {
                    intake.setIntakePower(0, 0);
                    intake.setIntakeServoPower(0);
                    outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false, false);
                })
                .waitSeconds(10);
        drivetrain.followTrajectorySequence(restOfIt4.build());
    }

    @Override
    public void loop() {
        drivetrain.update();
        outtake.periodic();
        if (clawSensor.autoSense()) {
            intake.setIntakePower(-0.2, 0);
        }
    }
}
