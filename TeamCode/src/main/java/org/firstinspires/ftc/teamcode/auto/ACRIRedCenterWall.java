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

@Autonomous(name = "A Red Center Wall CRI", group = " Testing") //aqua
@Config
public class ACRIRedCenterWall extends OpMode {
    static Pose2d startingPose = new Pose2d(-6.75, -61, Math.toRadians(90));  //starts agansed the pole
    static Pose2d finalPlacePos;
    static Pose2d finalPlacePos2 = new Pose2d(69, -38.5, Math.toRadians(180));

    static Pose2d pickupSpecial = new Pose2d(-77,-42, Math.toRadians(135));

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

    ConstantCRIPathsRed constantCRIPaths;
    ConstantCRIPathsRed.PlacePurplePaths placePurplePathsRed;
    ConstantCRIPathsRed.PickupWhitePixelStack pickupWhitePixelStack;
    ConstantCRIPathsRed.PlaceOnBackDrop placeOnBackDrop;


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
//        outtake.holdBackClaw(true);
        outtake.holdClaw(true);


        constantCRIPaths = new ConstantCRIPathsRed(telemetry, intake, outtake, cameras, clawSensor, drivetrain, specialIntake, new Pose2d(0,0, Math.toRadians(0)), new Pose2d(0,0, Math.toRadians(0)), new Vector2d(0,0));
        placePurplePathsRed = constantCRIPaths.placePurplePathsRed;
        pickupWhitePixelStack = constantCRIPaths.pickupWhitePixelStack;
        placeOnBackDrop = constantCRIPaths.placeOnBackDrop;

        //1**************************************************************************
        placeSpikeMark1 = drivetrain.trajectorySequenceBuilder(startingPose);
        placeSpikeMark1
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 55, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 55  //acc
                )
                .addTemporalMarker(() -> {
                    outtake.createPresetThread(Constants.Slides.autoPurple, Constants.Arm.autoArmDrop, 3, Constants.Extendo.auto, true, true);
                })
                .lineToLinearHeading(new Pose2d(-13, -34, Math.toRadians(0)))
                .waitSeconds(0.1)
                .addTemporalMarkerOffset(0.1, () -> {
                    outtake.holdBackClaw(false);
                })
                .addTemporalMarkerOffset(0.2, () -> {
                    outtake.createPresetThread(Constants.Slides.whileIntaking, Constants.Arm.intakePos, 3, false, true,false);
                })
                .waitSeconds(0.15)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(12, -55, Math.toRadians(180)));
        //2**************************************************************************
        placeSpikeMark2 = drivetrain.trajectorySequenceBuilder(startingPose);
        placeSpikeMark2
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 55, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 55  //acc
                )
                .lineToLinearHeading(new Pose2d(-9, -34, Math.toRadians(-90)))
                .addTemporalMarkerOffset(-0.5, () -> {
                    outtake.createPresetThread(Constants.Slides.autoPurple, Constants.Arm.autoArmDrop, 3, Constants.Extendo.auto, true, true);
                })
                .waitSeconds(0.2)
                .addTemporalMarkerOffset(0, () -> {
                    outtake.holdBackClaw(false);
                })
                .addTemporalMarkerOffset(0.15, () -> {
                    outtake.createPresetThread(Constants.Slides.whileIntaking, Constants.Arm.intakePos, 3, false, true,false);
                })
                .waitSeconds(0.15)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(12, -55, Math.toRadians(180)));
        //3**************************************************************************
        placeSpikeMark3 = drivetrain.trajectorySequenceBuilder(startingPose);
        placeSpikeMark3
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 30, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 30  //acc
                )
                .lineToLinearHeading(new Pose2d(-4, -46, Math.toRadians(-90)))
                .addTemporalMarkerOffset(-0.7, () -> {
                    outtake.createPresetThread(Constants.Slides.autoPurple, Constants.Arm.autoArmDrop, 3, Constants.Extendo.auto, true, true);
                })
                .waitSeconds(0.2)
                .resetConstraints()
                .addTemporalMarkerOffset(0, () -> {
                    outtake.holdBackClaw(false);
                })
                .addTemporalMarkerOffset(0.1, () -> {
                    outtake.createPresetThread(Constants.Slides.whileIntaking, Constants.Arm.intakePos, 3, false, true,false);
                })
                .waitSeconds(0.15)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(12, -55, Math.toRadians(180)));

        lights.setDumbLed(0);
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

        drivetrain.getPoseEstimateOptical(startingPose);

        TrajectorySequence placeSpikeMarkActual = null;

        if (barcodePosition == BarcodePosition.One) {
            placeSpikeMarkActual = placeSpikeMark1.build();
        } else if (barcodePosition == BarcodePosition.Two) {
            placeSpikeMarkActual = placeSpikeMark2.build();
        } else {
            placeSpikeMarkActual = placeSpikeMark3.build();
        }

        drivetrain.followTrajectorySequence(placeSpikeMarkActual);
        restOfIt = drivetrain.trajectorySequenceBuilder(placeSpikeMarkActual.end());

        if (barcodePosition == BarcodePosition.One) {
            finalPlacePos = new Pose2d(68, -33, Math.toRadians(180)); //right

        } else if (barcodePosition == BarcodePosition.Two) {
//            finalPlacePos = new Pose2d(68, -36.25, Math.toRadians(180)); //left
            finalPlacePos = new Pose2d(68, -39.75, Math.toRadians(180)); //right

        } else {//if barcodePosition == BarcodePosition.Three
//            finalPlacePos = new Pose2d(68, -43, Math.toRadians(180)); //left
            finalPlacePos = new Pose2d(68, -44.5, Math.toRadians(180)); //center
//            finalPlacePos = new Pose2d(68, -46, Math.toRadians(180)); //right

        }

        restOfIt
                .setReversed(true)
                .addTemporalMarkerOffset(0, () -> {
                    outtake.holdClaw(true);
                })
                .resetConstraints()
                .splineToSplineHeading(new Pose2d(60,-53, Math.toRadians(215)), Math.toRadians(0))
                .waitSeconds(0.5)
                .addTemporalMarkerOffset(0, () -> {
                    double distance = 30.0;
                    while (distance > 15.0) {
                        Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, true);
                        boolean present = optionalPose.isPresent();
                        if (present) {
                            distance = Math.sqrt(Math.pow(drivetrain.getPoseEstimateOptical().getX() - optionalPose.get().getX(), 2) + Math.pow(drivetrain.getPoseEstimateOptical().getY() - optionalPose.get().getY(), 2));
                            if (distance < 15.0) {
                                drivetrain.getPoseEstimateOptical(optionalPose.get());
                            }
                        }
                        telemetry.addData("distance = ", distance);
                        telemetry.addData("did see one", optionalPose.isPresent());
                        telemetry.update();
                    }
                })
                .splineToLinearHeading(finalPlacePos, Math.toRadians(0))
                .addTemporalMarkerOffset(-2, () -> {
                    intake.setIntakePower(0, 0);
                    intake.setIntakeServoPower(0);
                    outtake.createPresetThread(Constants.Slides.superLow + 150/*+200 for 2nd row*/, Constants.Arm.placePos, 1, Constants.Extendo.extended, true);
                })
                .addTemporalMarkerOffset(0, () -> {
                    outtake.holdClaw(false);
                    outtake.extend(false);
                    cameras.setCameraSideThreaded(false);
                })
                .addTemporalMarkerOffset(0.2, () -> {
                    intake.setIntakePower(0, 0);
                    intake.setIntakeServoPower(0);
                    outtake.createPresetThread(5, Constants.Arm.intakePos, 3, false, false);
                })
                //Placed on BD yellow
                .setReversed(false)
                .splineTo(new Vector2d(43, -60), Math.toRadians(180))
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 50, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 50  //acc
                )
                .splineTo(new Vector2d(-24, -60), Math.toRadians(180))
                .splineTo(new Vector2d(-58, -60), Math.toRadians(180))
                .resetConstraints()
                .splineToSplineHeading(new Pose2d(-65, -50, Math.toRadians(125)), Math.toRadians(125))
                .addTemporalMarkerOffset(0.25, () -> {
                    double distance = 30.0;
                    while (distance > 15.0) {
                        Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(7, false);
                        boolean present = optionalPose.isPresent();
                        if (present) {
                            distance = Math.sqrt(Math.pow(drivetrain.getPoseEstimateOptical().getX() - optionalPose.get().getX(), 2) + Math.pow(drivetrain.getPoseEstimateOptical().getY() - optionalPose.get().getY(), 2));
                            if (distance < 15.0) {
                                drivetrain.getPoseEstimateOptical(optionalPose.get());
                            }
                        }
                    }
                })
                .waitSeconds(0.5)
                //Picking up off stack
                .splineToSplineHeading(pickupSpecial, Math.toRadians(315))
                .addTemporalMarkerOffset(-0.2, () -> {
                    clawSensor.setRunInAuto(true);
                    outtake.presetSlides(Constants.Slides.whileIntaking);
                    outtake.setExtendo(Constants.Extendo.whileIntaking);
                    intake.setIntakePower(Constants.Intake.intake, 0);
                    intake.setIntakeServoPower(Constants.Intake.intakeServoIntake);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down4);
                })
                .addTemporalMarkerOffset(0.1, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.up);
                    cameras.setCameraSideThreaded(true);
                })
                .waitSeconds(0.1)
                .forward(4.5)
                .setReversed(true)
                .addTemporalMarkerOffset(2, () -> {
                    intake.setIntakePower(Constants.Intake.outake, 0);
                    outtake.presetSlides(0);
                    clawSensor.setRunInAuto(false);
                })
                .addTemporalMarkerOffset(3, () -> {
                    intake.setIntakePower(0, 0);
                })
                .lineToLinearHeading(new Pose2d(-60, -59, Math.toRadians(180)))
                .waitSeconds(0.1)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .splineTo(new Vector2d(30,-54), Math.toRadians(0))
                .addTemporalMarkerOffset(-2, () -> {
                    outtake.holdClaw(true);
                })
                .splineToSplineHeading(new Pose2d(60,-47, Math.toRadians(215)), Math.toRadians(0))
                .waitSeconds(0.2)
                .resetConstraints()
                .addTemporalMarkerOffset(0, () -> {
                    double distance = 30.0;
                    while (distance > 25.0) {
                        Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, true);
                        boolean present = optionalPose.isPresent();
                        if (present) {
                            distance = Math.sqrt(Math.pow(drivetrain.getPoseEstimateOptical().getX() - optionalPose.get().getX(), 2) + Math.pow(drivetrain.getPoseEstimateOptical().getY() - optionalPose.get().getY(), 2));
                            if (distance < 15.0) {
                                drivetrain.getPoseEstimateOptical(optionalPose.get());
                            }
                        }
                        telemetry.addData("distance = ", distance);
                        telemetry.addData("did see one", optionalPose.isPresent());
                        telemetry.update();
                    }
                })
                .waitSeconds(0.2)
                .splineToLinearHeading(finalPlacePos2, Math.toRadians(0))
                .addTemporalMarkerOffset(-2, () -> {
                    intake.setIntakePower(0, 0);
                    intake.setIntakeServoPower(0);
                    outtake.createPresetThread(Constants.Slides.low + 400, Constants.Arm.placePos, 5, Constants.Extendo.extended, true);
                })
                //Placing 2nd whites
                .addTemporalMarkerOffset(0, () -> {
                    outtake.holdClaw(false);
                    outtake.extend(false);
                })
                .setReversed(false)
                .addTemporalMarkerOffset(0.5, () -> {
                    intake.setIntakePower(0, 0);
                    intake.setIntakeServoPower(0);
                    outtake.createPresetThread(5, Constants.Arm.intakePos, 3, false, false);
                })
                .waitSeconds(10);

        drivetrain.followTrajectorySequenceAsync(restOfIt.build());
    }

    @Override
    public void loop() {
        drivetrain.update();
        outtake.periodic(false);
        if (clawSensor.autoSense()) {
            intake.setIntakePower(-0.5, 0);
            outtake.presetSlides(-5);
        }
    }
}
