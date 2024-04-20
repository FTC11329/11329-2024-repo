package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name = "Red Left 3 + 2 D C", group = "Competition")
@Config
public class RedLeft3DC extends OpMode {
    boolean whiteLeft;
    boolean hasTwo;
    static Pose2d startingPose = new Pose2d(-41, -60, Math.toRadians(90));
    static Vector2d placePositionOne = new Vector2d(52, - 33.25);
    static Vector2d placePositionTwo = new Vector2d(52, -37.25);
    static Vector2d placePositionThree = new Vector2d(52, -41.5);
    
    static Vector2d pickupSpecial = new Vector2d(-52.5, -12);
    static Vector2d pickupSpecial2 = new Vector2d(-56.5,-4.5);

    static double timeForPixelPlacement = 0.15;

    static boolean doWeCare = true;

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
                .lineToLinearHeading(new Pose2d(-59, -31, Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    outtake.presetArm(Constants.Arm.autoArmDrop);
                })
                .addTemporalMarkerOffset(timeForPixelPlacement, () -> {
                    outtake.presetArm(Constants.Arm.intakePos);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                })
                .waitSeconds(timeForPixelPlacement)
                .lineTo(pickupSpecial.plus(new Vector2d(-3.75,2)))
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
                .lineTo(pickupSpecial.plus(new Vector2d(-3.5,2)))
                .build();
        //3**************************************************************************
        placeSpikeMark3 = drivetrain.trajectorySequenceBuilder(startingPose)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .lineToLinearHeading(new Pose2d(-36, -29, Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    outtake.presetArm(Constants.Arm.autoArmDrop);
                })
                .addTemporalMarkerOffset(timeForPixelPlacement, () -> {
                    outtake.presetArm(Constants.Arm.intakePos);
                    specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                })
                .waitSeconds(timeForPixelPlacement)
                .lineTo(pickupSpecial.plus(new Vector2d(-4.75,0.5)))
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
        telemetry.addData("Front color", clawSensor.getFrontColor());
        telemetry.addData("Back Color" , clawSensor.getBackColor());
        telemetry.update();
    }

    @Override
    public void start() {
        cameras.setCameraSideThreaded(true);
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
            if (doWeCare) {
                finalPlaceLocation2 = placePositionOne.plus(new Vector2d(2, 2.5));
            } else {
                finalPlaceLocation2 = placePositionOne.plus(new Vector2d(2, -5));
            }
            whiteLeft = false;
        } else if (barcodePosition == BarcodePosition.Two) {
            finalPlaceLocation  = placePositionTwo;
            if (doWeCare) {
                finalPlaceLocation2 = placePositionOne.plus(new Vector2d(2, 2.5));
            } else {
                finalPlaceLocation2 = placePositionThree.plus(new Vector2d(2,-2.0));
            }
            whiteLeft = true;
        } else if (barcodePosition == BarcodePosition.Three) {
            finalPlaceLocation = placePositionThree;
            if (doWeCare) {
//                finalPlaceLocation2 = placePositionThree.plus(new Vector2d(2, 4)); hereeeeeeeeeeeeeeeeeeeeeeeee
                finalPlaceLocation2 = placePositionOne.plus(new Vector2d(2, 2.5));
            } else {
                finalPlaceLocation2 = placePositionThree.plus(new Vector2d(2, 2));
            }
            whiteLeft = true;
        } else return;


        drivetrain.followTrajectorySequenceAsync(drivetrain
                .trajectorySequenceBuilder(placeSpikeMark2.end())
                .resetConstraints()
                .setReversed(true)
                .addTemporalMarkerOffset(0, () -> {
                    clawSensor.setRunInAuto(true);
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
                .forward(3)
                .setReversed(true)
                .addTemporalMarkerOffset(1.5, () -> {
                    intake.setIntakePower(Constants.Intake.outake, 0);
                    claw.setPower(Constants.Claw.intake);
                })
                .addTemporalMarkerOffset(2, () -> {
                    claw.setPower(0);
                    clawSensor.setRunInAuto(false);
                    hasTwo = clawSensor.isFull();
                    telemetry.addData("now", true);
                    telemetry.update();
                    intake.setIntakePower(0, 0);
                })
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 65, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 65  //acc
                )
                .splineTo(new Vector2d(-5,-7), Math.toRadians(0))
                .splineTo(new Vector2d(30, -11), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45, -21), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    outtake.preset(Constants.Slides.superLow + 150, Constants.Arm.placePos);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    Optional<Pose2d> optionalPose = cameras.getRunnerPoseEstimate(0, true);
                    optionalPose.ifPresent(pose2d -> drivetrain.setPoseEstimate(pose2d));
                    telemetry.addData("has two"/*reminder*/, hasTwo);
                    telemetry.addData("did see one", optionalPose.isPresent());
                    telemetry.update();
                    cameras.setCameraSideThreaded(false);
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
                    outtake.presetSlides(Constants.Slides.superLow + 100);
                })

                .addTemporalMarkerOffset(0.6, () -> {
                    claw.setPower(Constants.Claw.outake);
                })
                .addTemporalMarkerOffset(0.9, () -> {
                    outtake.presetSlides(Constants.Slides.low);
                })
                .addTemporalMarkerOffset(1.5, () -> {
                    outtake.preset(Constants.Slides.intake, Constants.Arm.intakePos);
                    claw.setPower(0);
                })
                .waitSeconds(0.9)
                .setReversed(false)
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 55, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 55  //acc
                )
                .splineToConstantHeading(new Vector2d(36, -9.5), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-30, -9, Math.toRadians(195)), Math.toRadians(180))
                .waitSeconds(0.2)
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

                .resetConstraints()
                .lineToLinearHeading(new Pose2d(pickupSpecial2.getX(), pickupSpecial2.getY(), Math.toRadians(180)))
                .addTemporalMarkerOffset(-1.5, () -> {
                    intake.setIntakePower(Constants.Intake.intake, 0);
                    claw.setPower(Constants.Claw.intake);
                    clawSensor.setRunInAuto(true);
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
                .addTemporalMarkerOffset(0.3, () -> {
                    specialIntake.setIntakeServo(Constants.SpecialIntake.up);
                })
                .waitSeconds(0.3)
                .forward(2.5)
                .setReversed(true)
                .addTemporalMarkerOffset(2, () -> {
                    clawSensor.setRunInAuto(false);
                    intake.setIntakePower(Constants.Intake.outake, 0);
                    claw.setPower(0);
                })
                .addTemporalMarkerOffset(3, () -> {
                    intake.setIntakePower(0, 0);
                })
                .setConstraints(
                        (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                        (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                )
                .splineTo(new Vector2d(0,-7), Math.toRadians(0))
                .splineTo(new Vector2d(30, -10), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, -31), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    if (doWeCare) {
                        outtake.preset(Constants.Slides.med - 850, Constants.Arm.placePos);
                    } else {
                        outtake.preset(Constants.Slides.med - 400, Constants.Arm.placePos);
                    }
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
                .waitSeconds(0.4)
//                .forward(10) hereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee
                .lineTo(new Vector2d(47, -10))
                .addTemporalMarkerOffset(-0.5, () -> {
                    claw.setPower(0);
                    outtake.presetSlides(Constants.Slides.intake);
                    outtake.presetArm(Constants.Arm.intakePos);
                })

                .build());
    }

    @Override
    public void loop() {
        drivetrain.update();
        if (clawSensor.autoSense()) {
            intake.setIntakePower(Constants.Intake.outake, 0);
            claw.setPower(0);
        }
    }
}
