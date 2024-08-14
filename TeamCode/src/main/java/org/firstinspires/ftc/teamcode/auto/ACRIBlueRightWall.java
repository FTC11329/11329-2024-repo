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

@Autonomous(name = "A Blue Right Wall CRI", group = " Competition") //orange ~9.75 sec left
@Config
public class ACRIBlueRightWall extends OpMode {
    static Pose2d startingPose = new Pose2d(-64.25, 63, Math.toRadians(-90));
    static Pose2d finalPlacePos;

    static Pose2d pickupSpecial = new Pose2d(-79.5,39, Math.toRadians(180));

    int wristRot;

    TrajectorySequenceBuilder placeSpikeMark1 = null;
    TrajectorySequenceBuilder placeSpikeMark2 = null;
    TrajectorySequenceBuilder placeSpikeMark3 = null;

    TrajectorySequenceBuilder restOfIt = null;

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

        constantCRIPaths = new ConstantCRIPathsBlue(telemetry, intake, outtake, cameras, clawSensor, drivetrain, specialIntake, pickupSpecial, new Pose2d(0,0, Math.toRadians(0)), new Vector2d(0,0));
        placePurplePaths = constantCRIPaths.placePurplePathsBlue;
        pickupWhitePixelStack = constantCRIPaths.pickupWhitePixelStack;
        placeOnBackDrop = constantCRIPaths.placeOnBackDrop;

        //1**************************************************************************
        placeSpikeMark1 = drivetrain.trajectorySequenceBuilder(startingPose);
        placePurplePaths.RightPlacePos1Right.run(placeSpikeMark1);
        //2**************************************************************************
        placeSpikeMark2 = drivetrain.trajectorySequenceBuilder(startingPose);
        placePurplePaths.RightPlacePos2Right.run(placeSpikeMark2);
        //3**************************************************************************
        placeSpikeMark3 = drivetrain.trajectorySequenceBuilder(startingPose);
        placePurplePaths.RightPlacePos3Right.run(placeSpikeMark3);
        lights.setDumbLed(0);
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
    }

    @Override
    public void start() {
        cameras.setCameraSideThreaded(true);
        BarcodePosition barcodePosition = distanceSensors.getDirectionBlue(true);

        drivetrain.getPoseEstimateOptical(startingPose);

        TrajectorySequence placeSpikeMarkActual;

        if (barcodePosition == BarcodePosition.One) {
            placeSpikeMarkActual = placeSpikeMark1.build();
        } else if (barcodePosition == BarcodePosition.Two) {
            placeSpikeMarkActual = placeSpikeMark2.build();
        } else {
            placeSpikeMarkActual = placeSpikeMark3.build();
        }

        drivetrain.followTrajectorySequence(placeSpikeMarkActual);
        restOfIt = drivetrain.trajectorySequenceBuilder(placeSpikeMarkActual.end());
        restOfIt
                .addTemporalMarkerOffset(0, () -> {
                    clawSensor.setRunInAuto(true);
                    outtake.presetSlides(Constants.Slides.whileIntaking);
                    outtake.setExtendo(Constants.Extendo.whileIntaking);
                    intake.setIntakePower(Constants.Intake.intake, 0);
                    intake.setIntakeServoPower(Constants.Intake.intakeServoIntake);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down5);
                })
                .addTemporalMarkerOffset(0.4, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.up);
                })
                .waitSeconds(0.3)
                .forward(2.75)
                .setReversed(true)
                .addTemporalMarkerOffset(1.5, () -> {
                    intake.setIntakePower(Constants.Intake.outake, 0);
                    outtake.extend(false);
                    outtake.presetSlides(-20);
                    clawSensor.setRunInAuto(false);
                })
                .addTemporalMarkerOffset(3, () -> {
                    intake.setIntakePower(0, 0);
                })
                .lineTo(new Vector2d(-80, 55));

        if (barcodePosition == BarcodePosition.One) {
            finalPlacePos = new Pose2d(70, 41, Math.toRadians(180));
            wristRot = 5;
        } else if (barcodePosition == BarcodePosition.Two) {
            finalPlacePos = new Pose2d(70, 34.25, Math.toRadians(180));
            wristRot = 5;

        } else {//if barcodePosition == BarcodePosition.Three
            finalPlacePos = new Pose2d(70, 28.75, Math.toRadians(180));
            wristRot = 1;

        }
        restOfIt
                .addTemporalMarkerOffset(0, () -> {
                    cameras.setCameraSideThreaded(true);
                })
                .lineToSplineHeading(new Pose2d(-72, 56, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(-60, 56, Math.toRadians(180)), Math.toRadians(0))
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 55, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 55  //acc
                )
                .splineTo(new Vector2d(0,51), Math.toRadians(0))
                .splineTo(new Vector2d(50,51), Math.toRadians(0))
                .addTemporalMarkerOffset(-2.5, () -> {
                    outtake.presetSlides(-300);
                })
                .addTemporalMarkerOffset(-2, () -> {
                    outtake.holdClaw(true);
                })
                .resetConstraints()
                .splineToLinearHeading(new Pose2d(60,50, Math.toRadians(-200)), Math.toRadians(0))
                .waitSeconds(0.5)
                .addTemporalMarkerOffset(0, () -> {
                    double distance = 30.0;
                    while (distance > 25.0) {
                        Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, true);
                        boolean present = optionalPose.isPresent();
                        if (present) {
                            distance = Math.sqrt(Math.pow(drivetrain.getPoseEstimateOptical().getX() - optionalPose.get().getX(), 2) + Math.pow(drivetrain.getPoseEstimateOptical().getY() - optionalPose.get().getY(), 2));
                            if (distance < 25.0) {
                                drivetrain.getPoseEstimateOptical(optionalPose.get());
                            }
                        }
                        telemetry.addData("distance = ", distance);
                        telemetry.addData("did see one", optionalPose.isPresent());
                        telemetry.update();
                    }
                })
                .waitSeconds(0.2)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 26, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 26  //acc
                )
                .lineToLinearHeading(finalPlacePos)
                .addTemporalMarkerOffset(-2, () -> {
                    intake.setIntakePower(0, 0);
                    intake.setIntakeServoPower(0);
                    outtake.createPresetThread(Constants.Slides.low, Constants.Arm.placePos, 7, Constants.Extendo.extended, true);
                })
                .resetConstraints()
                .waitSeconds(0.2)
                .addTemporalMarkerOffset(0, () -> {
                    outtake.presetSlides(650);
                })
                .addTemporalMarkerOffset(0.2, () -> {
                    double distance = 30.0;
                    while (distance > 25.0) {
                        Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, true);
                        boolean present = optionalPose.isPresent();
                        if (present) {
                            distance = Math.sqrt(Math.pow(drivetrain.getPoseEstimateOptical().getX() - optionalPose.get().getX(), 2) + Math.pow(drivetrain.getPoseEstimateOptical().getY() - optionalPose.get().getY(), 2));
                            if (distance < 25.0) {
                                drivetrain.getPoseEstimateOptical(optionalPose.get());
                            }
                        }
                        telemetry.addData("distance = ", distance);
                        telemetry.addData("did see one", optionalPose.isPresent());
                        telemetry.update();
                    }
                })
                .addTemporalMarkerOffset(0.8, () -> {
                    outtake.holdClaw(false);
                    outtake.extend(false);
                })
                .waitSeconds(1)
                .setReversed(false)
                .addTemporalMarkerOffset(0.5, () -> {
                    intake.setIntakePower(0, 0);
                    intake.setIntakeServoPower(0);
                    outtake.createPresetThread(5, Constants.Arm.intakePos, 3, false, false);
                })
                // Left Park
//                .lineTo(new Vector2d(70, 64))
//                .lineTo(new Vector2d(85, 64));

                // Center
                .lineTo(new Vector2d(70, 40));

                // Right Park
//                .waitSeconds(0.5)
//                .lineTo(new Vector2d(74 , 19));

        restOfIt.waitSeconds(10);

        drivetrain.followTrajectorySequenceAsync(restOfIt.build());
    }

    @Override
    public void loop() {
        drivetrain.update();
        outtake.periodic();
        if (clawSensor.autoSense()) {
            intake.setIntakePower(-0.5, 0);
            outtake.presetSlides(-5);
        }
    }
}
