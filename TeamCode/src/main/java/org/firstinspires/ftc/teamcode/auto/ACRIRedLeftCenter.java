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

import javax.security.auth.callback.Callback;

@Autonomous(name = "A Red Left Center CRI", group = " Testing") //yellow
@Config
public class ACRIRedLeftCenter extends OpMode {
    int wristPos;
    static Pose2d startingPose = new Pose2d(-65.25, -63, Math.toRadians(90));
    static Vector2d finalPlacePos;
    static Vector2d finalPlacePos2 = new Vector2d(71.5, -8);

    static Pose2d pickupSpecial = new Pose2d(-72, -14.5, Math.toRadians(90));

    static Vector2d prePickup = new Vector2d(0, -15);
    static Pose2d pickupSpecial2 = new Pose2d(-11,-17.5, Math.toRadians(135)); //normal ***********************************************
    static double yComingBack = -18;
//    static Vector2d prePickup = new Vector2d(-24, -18);
//    static Pose2d pickupSpecial2 = new Pose2d(-34,-18, Math.toRadians(135)); //far    ***********************************************
//    static double yComingBack = -24;


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
        outtake.holdBackClaw(true);

        constantCRIPaths = new ConstantCRIPathsRed(telemetry, intake, outtake, cameras, clawSensor, drivetrain, specialIntake, new Pose2d(0,0, Math.toRadians(0)), new Pose2d(0,0, Math.toRadians(0)), new Vector2d(0,0));
        placePurplePathsRed = constantCRIPaths.placePurplePathsRed;
        pickupWhitePixelStack = constantCRIPaths.pickupWhitePixelStack;
        placeOnBackDrop = constantCRIPaths.placeOnBackDrop;

        //1**************************************************************************
        placeSpikeMark1 = drivetrain.trajectorySequenceBuilder(startingPose);
        placePurplePathsRed.LeftPlacePos1Center.run(placeSpikeMark1);
        //2**************************************************************************
        placeSpikeMark2 = drivetrain.trajectorySequenceBuilder(startingPose);
        placePurplePathsRed.LeftPlacePos2Center.run(placeSpikeMark2);
        //3**************************************************************************
        placeSpikeMark3 = drivetrain.trajectorySequenceBuilder(startingPose);
        placePurplePathsRed.LeftPlacePos3Center.run(placeSpikeMark3);
        lights.setDumbLed(0);
    }

    @Override
    public void init_loop() {
        boolean isBack = gamepad1.a;
        cameras.setCameraSide(isBack);

        BarcodePosition barcodePosition = distanceSensors.getDirectionRed(true);
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
        BarcodePosition barcodePosition = distanceSensors.getDirectionRed(true);

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
            finalPlacePos = new Vector2d(68.5, -32.75);
            wristPos = 5;

        } else if (barcodePosition == BarcodePosition.Two) {
            finalPlacePos = new Vector2d(68.5, -38);
            wristPos = 5;

        } else {//if barcodePosition == BarcodePosition.Three
            finalPlacePos = new Vector2d(68.5, -41.5);
            wristPos = 1;
        }

        drivetrain.followTrajectorySequence(placeSpikeMarkActual);
        restOfIt = drivetrain.trajectorySequenceBuilder(placeSpikeMarkActual.end());
        restOfIt
                .resetConstraints()
                .setReversed(true)
                .lineToLinearHeading(pickupSpecial)
                //1st pickup start
                .addTemporalMarkerOffset(-0.2, () -> {
                    clawSensor.setRunInAuto(true);
                    outtake.presetSlides(Constants.Slides.whileIntaking);
                    outtake.setExtendo(Constants.Extendo.whileIntaking);
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
                .lineToLinearHeading(new Pose2d(-65, -14, Math.toRadians(180)))
                .splineTo(new Vector2d(-12, -12), Math.toRadians(0))
                .splineTo(new Vector2d(40, -12), Math.toRadians(0))
                .waitSeconds(0.01)
                .addTemporalMarkerOffset(0, () -> {
                    double distance = 30.0;
                    while (distance > 15.0) {
                        Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, false);
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
                .splineToConstantHeading(new Vector2d(40, -15), Math.toRadians(180))
                .addTemporalMarkerOffset(0.05, () -> {
                    double distance = 30.0;
                    while (distance > 15.0) {
                        Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, false);
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
                .waitSeconds(0.1)
                .splineToConstantHeading(prePickup, Math.toRadians(180))
                .splineToSplineHeading(pickupSpecial2, Math.toRadians(180))
                //stack intake
                .addTemporalMarkerOffset(-0.5, () -> {
                    clawSensor.setRunInAuto(true);
                    outtake.presetSlides(Constants.Slides.whileIntaking);
                    outtake.setExtendo(Constants.Extendo.whileIntaking);
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
                .waitSeconds(0.01)
                .addTemporalMarkerOffset(0, () -> {
                    double distance = 30.0;
                    while (distance > 20.0) {
                        Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, false);
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
                .strafeLeft(13)
                .addTemporalMarkerOffset(-0.5, () -> {
                    intake.setIntakePower(0, 0);
                    intake.setIntakeServoPower(0);
                    outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false, false);
                })
                .waitSeconds(10);

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
