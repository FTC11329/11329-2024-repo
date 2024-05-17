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

@Autonomous(name = "Blue Right 3 + 2 D W", group = "Competition")
@Config
public class BlueRight3DW extends OpMode {
    boolean whiteLeft;
    boolean hasTwo;
    boolean doWeCare = true;

    static Pose2d startingPose = new Pose2d(-41, 60, Math.toRadians(-90));
    static Vector2d placePositionOne = new Vector2d(52.5, 42.5);
    static Vector2d placePositionTwo = new Vector2d(52.5, 38.25);
    static Vector2d placePositionThree = new Vector2d(52.5, 32);

    static Pose2d pickupSpecial  = new Pose2d(-55,36, Math.toRadians(180));
    static Pose2d pickupSpecial2 = new Pose2d(-59,43.25, Math.toRadians(204));


    static double timeForPixelPlacement = 0.15;

    TrajectorySequence placeSpikeMark1 = null;
    TrajectorySequence placeSpikeMark2 = null;
    TrajectorySequence placeSpikeMark3 = null;

    Intake intake;
    Outtake outtake;
    Cameras cameras;
    AutoServo autoServo;
    ClawSensor clawSensor;
    Drivetrain drivetrain;
    SpecialIntake specialIntake;
    DistanceSensors distanceSensors;

    public void init() {
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        cameras = new Cameras(hardwareMap);
        autoServo = new AutoServo(hardwareMap);
        clawSensor = new ClawSensor(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        specialIntake = new SpecialIntake(hardwareMap);
        distanceSensors = new DistanceSensors(hardwareMap);
        //1**************************************************************************
        placeSpikeMark1 = drivetrain.trajectorySequenceBuilder(startingPose)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .lineToLinearHeading(new Pose2d(-35, 30.5, Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    outtake.presetArm(Constants.Arm.autoArmDrop);
                })
                .addTemporalMarkerOffset(timeForPixelPlacement, () -> {
                    outtake.presetArm(Constants.Arm.intakePos);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                })
                .waitSeconds(timeForPixelPlacement)
                .lineToLinearHeading(pickupSpecial)
                .build();
        //2**************************************************************************
        placeSpikeMark2 = drivetrain.trajectorySequenceBuilder(startingPose)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .lineToLinearHeading(new Pose2d(-51, 20.5, Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    outtake.presetArm(Constants.Arm.autoArmDrop);
                })
                .addTemporalMarkerOffset(timeForPixelPlacement, () -> {
                    outtake.presetArm(Constants.Arm.intakePos);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                })
                .waitSeconds(timeForPixelPlacement)
                .lineToLinearHeading(pickupSpecial)
                .build();
        //3**************************************************************************
        placeSpikeMark3 = drivetrain.trajectorySequenceBuilder(startingPose)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .lineToLinearHeading(new Pose2d(-60, 20, Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    outtake.presetArm(Constants.Arm.autoArmDrop);
                })
                .addTemporalMarkerOffset(timeForPixelPlacement, () -> {
                    outtake.presetArm(Constants.Arm.intakePos);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                })
                .waitSeconds(timeForPixelPlacement)
                .lineToLinearHeading(pickupSpecial)
                .build();
    }

    @Override
    public void init_loop() {
        boolean isBack = gamepad1.a;
        cameras.setCameraSide(gamepad1.a);

        BarcodePosition barcodePosition = distanceSensors.getDirectionBlue(true);
        telemetry.addData("Barcode Position", barcodePosition);
        telemetry.addData("FPS", cameras.switchingCamera.getFps());
        telemetry.addData("Is back", isBack);
        telemetry.update();
    }

    @Override
    public void start() {
        cameras.setCameraSideThreaded(true);
        BarcodePosition barcodePosition = distanceSensors.getDirectionBlue(true);

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
            if (doWeCare) {
                finalPlaceLocation2 = placePositionOne.plus(new Vector2d(2,0.5));
            } else {
                finalPlaceLocation2 = placePositionOne.plus(new Vector2d(2, -5));
            }
            whiteLeft = false;
        } else if (barcodePosition == BarcodePosition.Two) {
            finalPlaceLocation  = placePositionTwo;
            if (doWeCare) {
                finalPlaceLocation2 = placePositionOne.plus(new Vector2d(2,0.5));
            } else {
                finalPlaceLocation2 = placePositionOne.plus(new Vector2d(2,1));
            }
            whiteLeft = true;
        } else if (barcodePosition == BarcodePosition.Three) {
            finalPlaceLocation = placePositionThree;
            if (doWeCare) {
                finalPlaceLocation2 = placePositionOne.plus(new Vector2d(2,0.5));
            } else {
                finalPlaceLocation2 = placePositionThree.plus(new Vector2d(2, 8));
            }
            whiteLeft = true;
        } else return;


        drivetrain.followTrajectorySequenceAsync(drivetrain
                .trajectorySequenceBuilder(placeSpikeMark2.end())
                .resetConstraints()
                .setReversed(true)
                .addTemporalMarkerOffset(0, () -> {
                    outtake.presetArm(Constants.Arm.intakePos);
                    intake.setIntakePower(Constants.Intake.intake, 0);
                })
                .addTemporalMarkerOffset(0.1, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.down5);
                })
                .addTemporalMarkerOffset(0.6, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.up);
                    clawSensor.setRunInAuto(true);
                })
                .waitSeconds(0.5)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 45, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 45  //acc
                )
                .forward(4)
                .setReversed(true)
                .addTemporalMarkerOffset(1.5, () -> {
                    intake.setIntakePower(Constants.Intake.outake, 0);
                })
                .addTemporalMarkerOffset(2, () -> {
                    clawSensor.setRunInAuto(false);
                    hasTwo = clawSensor.isFull();
                    telemetry.addData("now", true);
                    telemetry.update();
                    intake.setIntakePower(0, 0);
                })
                .splineToLinearHeading(new Pose2d(-35, 53.75, Math.toRadians(180)), Math.toRadians(0))
                .splineTo(new Vector2d(16, 50.75), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(35, 35), Math.toRadians(0))
                .addTemporalMarkerOffset(-0.5, () -> {
                    outtake.preset(Constants.Slides.superLow + 150, Constants.Arm.placePos);
                })
                .addTemporalMarkerOffset(0.25, () -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, true);
                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
                    telemetry.addData("did see one", optionalPose.isPresent());
                    telemetry.update();
                    cameras.setCameraSideThreaded(false);
                })
                .resetConstraints()
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 35, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 35  //acc
                )
                .waitSeconds(0.4)
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
                    }
                })
                .addTemporalMarkerOffset(0.12, () -> {
                    autoServo.upBoth();
                    outtake.presetSlides(Constants.Slides.superLow - 100);
                })

                .addTemporalMarkerOffset(0.6, () -> {
                })
                .addTemporalMarkerOffset(0.9, () -> {
                    outtake.presetSlides(Constants.Slides.low);
                })
                .addTemporalMarkerOffset(1.5, () -> {
                    outtake.preset(Constants.Slides.intake, Constants.Arm.intakePos);
                })
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 40, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 40  //acc
                )
                .waitSeconds(0.9)
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(16, 60), Math.toRadians(180))
                .splineTo(new Vector2d(-25, 58), Math.toRadians(180))
                .splineTo(new Vector2d(-40, 53), Math.toRadians(-160))
                .waitSeconds(0.3)
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
                        telemetry.addData("did see two", optionalPose.isPresent());
                        telemetry.update();
                    }
                    cameras.setCameraSideThreaded(true);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                })

                //Back For Another One**************************************
                .lineToLinearHeading(pickupSpecial2,
                        (displacement, pose, derivative, baseRobotVelocity) -> 30, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 30  //acc
                )
                .addTemporalMarkerOffset(-0.5, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                })
                .addTemporalMarkerOffset(0, () -> {
                    if (hasTwo) {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.down4);
                    } else {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.down5);
                    }
                    intake.setIntakePower(Constants.Intake.intake, 0);
                    outtake.presetArm(Constants.Arm.intakePos);
                })
                .addTemporalMarkerOffset(0.25, () -> {
                    if (hasTwo) {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.down3);
                    } else {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.down4);
                    }
                })
                .addTemporalMarkerOffset(0.75, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.up);
                })
                .waitSeconds(0.75)
                .forward(2.5)
                .waitSeconds(1) //HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
                .addTemporalMarkerOffset(0.25, () -> {
                    clawSensor.setRunInAuto(true);
                })
                .setReversed(true)
                .addTemporalMarkerOffset(2, () -> {
                    clawSensor.setRunInAuto(false);
                    intake.setIntakePower(Constants.Intake.outake, 0);
                })
                .addTemporalMarkerOffset(3, () -> {
                    intake.setIntakePower(0, 0);
                })
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 45, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 45  //acc
                )
                .splineTo(new Vector2d(-35, 56), Math.toRadians(0))
                .splineTo(new Vector2d(16, 56.5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(finalPlaceLocation2.getX() - 17, finalPlaceLocation2.getY()), Math.toRadians(0))
                .addTemporalMarkerOffset(-1, () -> {
                    outtake.preset(Constants.Slides.med - 800, Constants.Arm.placePos);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, true);
                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
                    telemetry.addData("did see Three", optionalPose.isPresent());
                    telemetry.update();
//                    cameras.kill();
                })
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 35, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 35  //acc
                )
                .lineTo(new Vector2d(finalPlaceLocation2.getX(), finalPlaceLocation2.getY()))
                .addTemporalMarkerOffset(-0.3, () -> {
                })
                .resetConstraints()
                .forward(10)
                .addTemporalMarkerOffset(-0.5, () -> {
                    outtake.presetSlides(Constants.Slides.intake);
                    outtake.presetArm(Constants.Arm.intakePos);
                })
                .addTemporalMarkerOffset(0, () -> {
                    cameras.kill();
                })
                .lineToLinearHeading(new Pose2d(49,66, Math.toRadians(180)))
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