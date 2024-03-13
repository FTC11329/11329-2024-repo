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
    boolean whiteLeft;
    boolean hasTwo;
    static Pose2d startingPose = new Pose2d(-41, -60, Math.toRadians(90));
    static Vector2d placePositionOne = new Vector2d(52, -33.5);
    static Vector2d placePositionTwo = new Vector2d(52, -35.5);
    static Vector2d placePositionThree = new Vector2d(52, -40.5);
    
    static Vector2d pickupSpecial = new Vector2d(-52.5, -12);
    static Vector2d pickupSpecial2 = new Vector2d(-56.75,-4.5);

    static double timeForPixelPlacement = 0.15;

    TrajectorySequence placeSpikeMark1 = null;
    TrajectorySequence placeSpikeMark2 = null;
    TrajectorySequence placeSpikeMark3 = null;

    TrajectorySequence restOfIt = null;

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
        //1**************************************************************************
        placeSpikeMark1 = drivetrain.trajectorySequenceBuilder(startingPose)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .lineToLinearHeading(new Pose2d(-58, -29, Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    outtake.presetArm(Constants.Arm.autoArmDrop);
                })
                .addTemporalMarkerOffset(timeForPixelPlacement, () -> {
                    outtake.presetArm(Constants.Arm.intakePos);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                })
                    .waitSeconds(timeForPixelPlacement)
                .lineTo(pickupSpecial.plus(new Vector2d(-3.5,2)))
                .build();
        //2**************************************************************************
        placeSpikeMark2 = drivetrain.trajectorySequenceBuilder(startingPose)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .lineToLinearHeading(new Pose2d(-53, -21.5, Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    outtake.presetArm(Constants.Arm.autoArmDrop);
                })
                .addTemporalMarkerOffset(timeForPixelPlacement, () -> {
                    outtake.presetArm(Constants.Arm.intakePos);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                })
                .waitSeconds(timeForPixelPlacement)
                .lineTo(pickupSpecial.plus(new Vector2d(-3,2)))
                .build();
        //3**************************************************************************
        placeSpikeMark3 = drivetrain.trajectorySequenceBuilder(startingPose)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .lineToLinearHeading(new Pose2d(-36, -32, Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    outtake.presetArm(Constants.Arm.autoArmDrop);
                })
                .addTemporalMarkerOffset(timeForPixelPlacement, () -> {
                    outtake.presetArm(Constants.Arm.intakePos);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                })
                .waitSeconds(timeForPixelPlacement)
                .lineTo(pickupSpecial.plus(new Vector2d(-4,-1)))
                .build();
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
            finalPlaceLocation2 = placePositionOne.plus(new Vector2d(5,-1));
            whiteLeft = false;
        } else if (barcodePosition == BarcodePosition.Two) {
            finalPlaceLocation  = placePositionTwo;
            finalPlaceLocation2 = placePositionThree.plus(new Vector2d(3,0));
            whiteLeft = true;
        } else if (barcodePosition == BarcodePosition.Three) {
            finalPlaceLocation  = placePositionThree;
            finalPlaceLocation2 = placePositionThree.plus(new Vector2d(3,4));
            whiteLeft = true;
        } else return;


        drivetrain.followTrajectorySequence(drivetrain
                .trajectorySequenceBuilder(placeSpikeMark2.end())
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
                .waitSeconds(0.5)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 65, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 65  //acc
                )
                .forward(4)
                .setReversed(true)
                .addTemporalMarkerOffset(2, () -> {
                    intake.setIntakePower(Constants.Intake.outake, 0);
                    claw.setPower(Constants.Intake.intake);
                })
                .addTemporalMarkerOffset(3, () -> {
                    claw.setPower(0);
                    hasTwo = clawSensor.isFull();
                    telemetry.addData("now", true);
                    intake.setIntakePower(0, 0);
                })
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 65, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 65  //acc
                )
                .splineTo(new Vector2d(25, -5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, -21), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    outtake.preset(Constants.Slides.superLow, Constants.Arm.placePos);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, true);
                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
                    telemetry.addData("has two", hasTwo);
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
                    if (whiteLeft) {
                        autoServo.DropLeft();
                    } else {
                        autoServo.DropRight();
                    }
                })
                .addTemporalMarkerOffset(-0.05, () -> {
                    if (hasTwo) {
                        claw.setPower(Constants.Claw.slowOutake);
                    }
                })
                .addTemporalMarkerOffset(0.12, () -> {
                    claw.setPower(0);
                    autoServo.upBoth();
                    outtake.presetSlides(Constants.Slides.superLow - 100);
                })

                .addTemporalMarkerOffset(0.6, () -> {
                    claw.setPower(Constants.Claw.outake);
                })
                .addTemporalMarkerOffset(0.9, () -> {
                    outtake.presetSlides(Constants.Slides.low);
                })
                .addTemporalMarkerOffset(1.5, () -> {
                    outtake.preset(Constants.Slides.intake, 0);
                    claw.setPower(0);
                })
                .waitSeconds(0.9)
                .setReversed(false)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 80  //acc
                )
                .splineToConstantHeading(new Vector2d(36, -9), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-30, -9, Math.toRadians(195)), Math.toRadians(180))
                .waitSeconds(0.5)
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
                .addTemporalMarkerOffset(-1.5, () -> {
                    intake.setIntakePower(Constants.Intake.intake, 0);
                    claw.setPower(Constants.Claw.intake);
                    if (hasTwo) {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.down4);
                    } else {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.down5);
                    }
                })

                .addTemporalMarkerOffset(0, () -> {
                    if (hasTwo) {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.down3);
                    } else {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.down4);
                    }
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
                        (displacement, pose, derivative, baseRobotVelocity) -> 65, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 65  //acc
                )
                .splineTo(new Vector2d(-11, -9), Math.toRadians(0))
                .splineTo(new Vector2d(25, -10), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(37, -31), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    outtake.preset(Constants.Slides.med - 850, Constants.Arm.placePos);
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
                        (displacement, pose, derivative, baseRobotVelocity) -> 30, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 30  //acc
                )
                .lineToLinearHeading(new Pose2d(finalPlaceLocation2.getX(), finalPlaceLocation2.getY(), Math.toRadians(180)))
                .addTemporalMarkerOffset(0, () -> {
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
