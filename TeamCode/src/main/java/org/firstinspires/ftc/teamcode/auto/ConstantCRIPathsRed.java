package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Cameras;
import org.firstinspires.ftc.teamcode.subsystems.ClawSensor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.SpecialIntake;
import org.firstinspires.ftc.teamcode.utility.SequenceFunction;

import java.util.Optional;


public class ConstantCRIPathsRed {
    Pose2d pickupSpecial;
    Pose2d pickupSpecial2;

    Vector2d centerBDPlacePos;

    Vector2d finalPlacePos1Center = new Vector2d(70, -32.25);
    Vector2d finalPlacePos2Center = new Vector2d(70, -32.25);
    Vector2d finalPlacePos3Center = new Vector2d(70, -37);

    Vector2d finalPlacePos1Left = new Vector2d(74, -38.25);
    Vector2d finalPlacePos2Left = new Vector2d(74, -38.25);
    Vector2d finalPlacePos3Left = new Vector2d(74, -43);

    public PlacePurplePaths placePurplePathsRed;
    public PickupWhitePixelStack pickupWhitePixelStack;
    public PlaceOnBackDrop placeOnBackDrop;

    public ConstantCRIPathsRed(Telemetry telemetry, Intake intake, Outtake outtake, Cameras cameras, ClawSensor clawSensor, Drivetrain drivetrain, SpecialIntake specialIntake, Pose2d pickupSpecial, Pose2d pickupSpecial2, Vector2d centerBDPlacePos) {
        placePurplePathsRed = new PlacePurplePaths(intake, outtake, cameras, clawSensor, drivetrain, specialIntake);
        pickupWhitePixelStack = new PickupWhitePixelStack(intake, outtake, cameras, clawSensor, drivetrain, specialIntake);
        placeOnBackDrop = new PlaceOnBackDrop(telemetry, intake, outtake, cameras, clawSensor, drivetrain, specialIntake);

        this.pickupSpecial = pickupSpecial;
        this.pickupSpecial2 = pickupSpecial2;
        this.centerBDPlacePos = centerBDPlacePos;
    }
    @Config
    public class PlacePurplePaths {
        private Intake intake;
        private Outtake outtake;
        private Cameras cameras;
        private ClawSensor clawSensor;
        private Drivetrain drivetrain;
        private SpecialIntake specialIntake;
        public PlacePurplePaths(Intake intake, Outtake outtake, Cameras cameras, ClawSensor clawSensor, Drivetrain drivetrain, SpecialIntake specialIntake) {
            this.intake = intake;
            this.outtake = outtake;
            this.cameras = cameras;
            this.clawSensor = clawSensor;
            this.drivetrain = drivetrain;
            this.specialIntake = specialIntake;
        }
            //Left *************************************************************************************************************************************************************************************************************************************

         SequenceFunction LeftPlacePos1Center = (prev) -> {//untested
            prev
                    .setConstraints(
                            (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                            (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                    )
                    .addTemporalMarker(() -> {
                        outtake.createPresetThread(150, Constants.Arm.autoArmDrop, 3, Constants.Extendo.auto, true);
                    })
                    .lineToLinearHeading(new Pose2d(-63, -31, Math.toRadians(0)))
                    .waitSeconds(0.1)
                    .addTemporalMarkerOffset(0.1, () -> {
                        outtake.holdClaw(false);
                    })
                    .addTemporalMarkerOffset(0.15, () -> {
                        outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false,false);
                    })
                    .addTemporalMarkerOffset(0.5, () -> {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                    })
                    .waitSeconds(0.15)
                    .turn(Math.toRadians(90))
                    .lineToLinearHeading((new Pose2d(-61,-15, Math.toRadians(180))));

        };
        SequenceFunction LeftPlacePos2Center = (prev) -> {//untested
            prev
                    .setConstraints(
                            (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                            (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                    )
                    .addTemporalMarker(() -> {
                        outtake.createPresetThread(150, Constants.Arm.autoArmDrop, 3, Constants.Extendo.auto, true);
                    })
                    .lineToLinearHeading(new Pose2d(-60, -15, Math.toRadians(90)))
                    .waitSeconds(0.1)
                    .addTemporalMarkerOffset(0.1, () -> {
                        outtake.holdClaw(false);
                    })
                    .addTemporalMarkerOffset(0.15, () -> {
                        outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false,false);
                    })
                    .addTemporalMarkerOffset(0.5, () -> {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                    })
                    .waitSeconds(0.15)
                    .lineToLinearHeading((new Pose2d(-61,-15, Math.toRadians(180))));
        };
        SequenceFunction LeftPlacePos3Center = (prev) -> { //untested
            prev
                    .setConstraints(
                            (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                            (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                    )
                    .addTemporalMarker(() -> {
                        outtake.createPresetThread(150, Constants.Arm.autoArmDrop, 3, Constants.Extendo.auto, true);
                    })
                    .lineToLinearHeading(new Pose2d(-62, -33, Math.toRadians(180)))
                    .waitSeconds(0.1)
                    .addTemporalMarkerOffset(0.1, () -> {
                        outtake.holdClaw(false);
                    })
                    .addTemporalMarkerOffset(0.15, () -> {
                        outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false,false);
                    })
                    .waitSeconds(0.15)
                    .turn(Math.toRadians(-90))
                    .addTemporalMarkerOffset(0, () -> {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                    })
                    .lineToLinearHeading((new Pose2d(-61,-15, Math.toRadians(180))));

        };

        SequenceFunction LeftPlacePos1Left = (prev) -> {//untested
            prev
                    .setConstraints(
                            (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                            (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                    )
                    .addTemporalMarker(() -> {
                        outtake.createPresetThread(150, Constants.Arm.autoArmDrop, 3, Constants.Extendo.auto, true);
                    })
                    .lineToLinearHeading(new Pose2d(-81, -35, Math.toRadians(180)))
                    .waitSeconds(0.1)
                    .addTemporalMarkerOffset(0.1, () -> {
                        outtake.holdClaw(false);
                    })
                    .addTemporalMarkerOffset(0.15, () -> {
                        outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false,false);
                    })
                    .addTemporalMarkerOffset(0.5, () -> {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                    })
                    .waitSeconds(0.15)
                    .turn(Math.toRadians(90))
                    .lineToLinearHeading((new Pose2d(-80,-35, Math.toRadians(180))));

        };
        SequenceFunction LeftPlacePos2Left = (prev) -> {//untested
            prev
                    .setConstraints(
                            (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                            (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                    )
                    .addTemporalMarker(() -> {
                        outtake.createPresetThread(150, Constants.Arm.autoArmDrop, 3, Constants.Extendo.auto, true);
                    })
                    .lineToLinearHeading(new Pose2d(-81, -24.5, Math.toRadians(180)))
                    .waitSeconds(0.1)
                    .addTemporalMarkerOffset(0.1, () -> {
                        outtake.holdClaw(false);
                    })
                    .addTemporalMarkerOffset(0.15, () -> {
                        outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false,false);
                    })
                    .addTemporalMarkerOffset(0.5, () -> {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                    })
                    .waitSeconds(0.15)
                    .lineToLinearHeading((new Pose2d(-80,-35, Math.toRadians(180))));
        };
        SequenceFunction LeftPlacePos3Left = (prev) -> { //untested
            prev
                    .setConstraints(
                            (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                            (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                    )
                    .addTemporalMarker(() -> {
                        outtake.createPresetThread(150, Constants.Arm.autoArmDrop, 3, Constants.Extendo.auto, true);
                    })
                    .lineToLinearHeading(new Pose2d(-62, -35, Math.toRadians(180)))
                    .waitSeconds(0.1)
                    .addTemporalMarkerOffset(0.1, () -> {
                        outtake.holdClaw(false);
                    })
                    .addTemporalMarkerOffset(0.15, () -> {
                        outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false,false);
                    })
                    .waitSeconds(0.15)
                    .addTemporalMarkerOffset(0, () -> {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                    })
                    .lineToLinearHeading((new Pose2d(-80,-35, Math.toRadians(180))));

        };
        //Center **************************************************************************************************************************************************************************************************************************************

        SequenceFunction CenterPlacePos1 = (prev) -> {
            prev
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
                    })
                    .addTemporalMarkerOffset(0.5, () -> {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                    })
                    .waitSeconds(0.15)
                    .turn(Math.toRadians(90))
                    .lineToLinearHeading(pickupSpecial.plus(new Pose2d(4.5,0, 0)));
        };
        SequenceFunction CenterPlacePos2 = (prev) -> {
            prev
                    .lineToSplineHeading(new Pose2d(-13, -36, Math.toRadians(180)))
                    .lineToSplineHeading(new Pose2d(-12.5, -24, Math.toRadians(180)))
                    .lineToSplineHeading(new Pose2d(-16, -14, Math.toRadians(145)))

                    .addTemporalMarkerOffset(-0.5, () -> {
                        outtake.createPresetThread(150, Constants.Arm.autoArmDrop, 3, Constants.Extendo.auto, true);
                    })
                    .waitSeconds(0.2)
                    .addTemporalMarkerOffset(0, () -> {
                        outtake.holdClaw(false);
                    })
                    .addTemporalMarkerOffset(0.1, () -> {
                        outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false,false);
                    })
                    .addTemporalMarkerOffset(0.5, () -> {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                    })
                    .waitSeconds(0.15)
                    .lineToLinearHeading(pickupSpecial.plus(new Pose2d(4.5,0, 125)));
        };
        SequenceFunction CenterPlacePos3 = (prev) -> {
            prev
                    .setConstraints(
                            (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                            (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                    )
                    .addTemporalMarker(() -> {
                        outtake.createPresetThread(150, Constants.Arm.autoArmDrop, 3, Constants.Extendo.auto, true);
                    })
                    .lineToLinearHeading(new Pose2d(-14, -33, Math.toRadians(180)))
                    .waitSeconds(0.1)
                    .addTemporalMarkerOffset(0.1, () -> {
                        outtake.holdClaw(false);
                    })
                    .addTemporalMarkerOffset(0.15, () -> {
                        outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false,false);
                    })
                    .waitSeconds(0.15)
                    .turn(Math.toRadians(-90))
                    .addTemporalMarkerOffset(0, () -> {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.ready);
                    })
                    .splineToLinearHeading(pickupSpecial.plus(new Pose2d(4,0, 0)), Math.toRadians(90));
        };
        //Right **************************************************************************************************************************************************************************************************************************************

        SequenceFunction RightPlacePos1 = (prev) -> {
            prev
                    .waitSeconds(1);
        };
        SequenceFunction RightPlacePos2 = (prev) -> {
            prev
                    .waitSeconds(1);
        };
        SequenceFunction RightPlacePos3 = (prev) -> {
            prev
                    .waitSeconds(1);
        };
    }
    @Config
    public class PickupWhitePixelStack {
        private Intake intake;
        private Outtake outtake;
        private Cameras cameras;
        private ClawSensor clawSensor;
        private Drivetrain drivetrain;
        private SpecialIntake specialIntake;
        public PickupWhitePixelStack(Intake intake, Outtake outtake, Cameras cameras, ClawSensor clawSensor, Drivetrain drivetrain, SpecialIntake specialIntake) {
            this.intake = intake;
            this.outtake = outtake;
            this.cameras = cameras;
            this.clawSensor = clawSensor;
            this.drivetrain = drivetrain;
            this.specialIntake = specialIntake;
        }
        SequenceFunction Left1stToCenterStack = (prev) -> { //untested
            prev
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
                    .forward(1.73)
                    .setReversed(true)
                    .addTemporalMarkerOffset(1.5, () -> {
                        intake.setIntakePower(Constants.Intake.outake, 0);
                        outtake.presetSlides(0);
                        clawSensor.setRunInAuto(false);
                        outtake.presetSlides(-1);
                    })
                    .addTemporalMarkerOffset(3, () -> {
                        intake.setIntakePower(0, 0);
                    });
        };
        SequenceFunction Left1stToWallStack = (prev) -> {//untested
            prev
                    .lineToLinearHeading(pickupSpecial)
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
                    .waitSeconds(0.1)
                    .forward(1.73)
                    .setReversed(true)
                    .addTemporalMarkerOffset(1.5, () -> {
                        intake.setIntakePower(Constants.Intake.outake, 0);
                        outtake.presetSlides(-20);
                        clawSensor.setRunInAuto(false);
                    })
                    .addTemporalMarkerOffset(3, () -> {
                        intake.setIntakePower(0, 0);
                    });
        };

        SequenceFunction BackDropToWallStack = (prev) -> {
            prev
                    .splineTo(new Vector2d(43, -60), Math.toRadians(180))
                    .addTemporalMarkerOffset(-0.2, () -> {
                        cameras.setCameraSideThreaded(false);
                    })
                    .setConstraints(
                            (displacement, pose, derivative, baseRobotVelocity) -> 55, //vel
                            (displacement, pose, derivative, baseRobotVelocity) -> 55  //acc
                    )
                    .lineTo(new Vector2d(-72, -55))
                    .lineToLinearHeading(new Pose2d(-73, -55, Math.toRadians(180)))
                    .resetConstraints()
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
                        }
                    })
                    .waitSeconds(0.5)
                    .splineToSplineHeading(pickupSpecial2, Math.toRadians(315))
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
                    .waitSeconds(0.1)
                    .forward(1.73)
                    .setReversed(true)
                    .addTemporalMarkerOffset(1.5, () -> {
                        intake.setIntakePower(Constants.Intake.outake, 0);
                        outtake.presetSlides(0);
                        clawSensor.setRunInAuto(false);
                    })
                    .addTemporalMarkerOffset(3, () -> {
                        intake.setIntakePower(0, 0);
                    });

        };

        SequenceFunction Center1stToCenterStack = (prev) -> {
            prev
                    .resetConstraints()
                    .setReversed(true)
                    .lineToLinearHeading(pickupSpecial)
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
                    .waitSeconds(0.1)
                    .forward(1.73)
                    .setReversed(true)
                    .addTemporalMarkerOffset(1.5, () -> {
                        intake.setIntakePower(Constants.Intake.outake, 0);
                        outtake.presetSlides(0);
                        clawSensor.setRunInAuto(false);
                    })
                    .addTemporalMarkerOffset(3, () -> {
                        intake.setIntakePower(0, 0);
                    });
        };
        SequenceFunction BackDropToCenterStack = (prev) -> {
            prev
                    .splineTo(new Vector2d(40, -13), Math.toRadians(180))
                    .splineTo(new Vector2d(0, -13), Math.toRadians(180))
                    .splineToSplineHeading(pickupSpecial2, Math.toRadians(180))
                    .addTemporalMarkerOffset(-0.5, () -> {
                        clawSensor.setRunInAuto(true);
                        outtake.presetSlides(Constants.Slides.whileIntaking);
                        intake.setIntakePower(Constants.Intake.intake, 0);
                        intake.setIntakeServoPower(Constants.Intake.intakeServoIntake);
                        specialIntake.setIntakeServo(Constants.SpecialIntake.down4);
                    })
                    .addTemporalMarkerOffset(0.05 , () -> {
                        intake.setIntakePower(Constants.Intake.intake, 0);
                        specialIntake.setIntakeServo(Constants.SpecialIntake.down1);

                    })

                    .waitSeconds(0.1)
                    .forward(1.73)
                    .addTemporalMarkerOffset(0.5 , () -> {
                        outtake.presetSlides(0);
                    })
                    .setReversed(true)
                    .addTemporalMarkerOffset(1.5, () -> {
                        intake.setIntakePower(Constants.Intake.outake, 0);
                        clawSensor.setRunInAuto(false);
                    })
                    .addTemporalMarkerOffset(3, () -> {
                        intake.setIntakePower(0, 0);
                    });
        };
    }
    @Config
    public class PlaceOnBackDrop {
        private Telemetry telemetry;
        private Intake intake;
        private Outtake outtake;
        private Cameras cameras;
        private ClawSensor clawSensor;
        private Drivetrain drivetrain;
        private SpecialIntake specialIntake;
        public PlaceOnBackDrop(Telemetry telemetry, Intake intake, Outtake outtake, Cameras cameras, ClawSensor clawSensor, Drivetrain drivetrain, SpecialIntake specialIntake) {
            this.telemetry = telemetry;
            this.intake = intake;
            this.outtake = outtake;
            this.cameras = cameras;
            this.clawSensor = clawSensor;
            this.drivetrain = drivetrain;
            this.specialIntake = specialIntake;
        }
        //1*****************************************************************************************
        SequenceFunction WallStackTo1stPlacePos = (prev) -> {//untested
            prev
                    .lineToSplineHeading(new Pose2d(-72, -50, Math.toRadians(0)))
                    .splineToSplineHeading(new Pose2d(-48, -57, Math.toRadians(0)), Math.toRadians(0))
                    .setConstraints(
                            (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                            (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                    )
                    .lineTo(new Vector2d(-36,-57))
                    .resetConstraints()
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
                    .splineTo(finalPlacePos1Left, Math.toRadians(0))//final place pos 1*******************
                    .addTemporalMarkerOffset(-2, () -> {
                        intake.setIntakePower(0, 0);
                        intake.setIntakeServoPower(0);
                        outtake.createPresetThread(Constants.Slides.superLow, Constants.Arm.placePos, 5, Constants.Extendo.half, true);
                    })
                    .addTemporalMarkerOffset(0.2, () -> {
                        outtake.holdClaw(false);
                        outtake.extend(false);
                    })
                    .setReversed(false)
                    .addTemporalMarkerOffset(0.05, () -> {
                        intake.setIntakePower(0, 0);
                        intake.setIntakeServoPower(0);
                        outtake.createPresetThread(5, Constants.Arm.intakePos, 3, false, false);
                    });
        };
        SequenceFunction WallStackTo2ndPlacePos = (prev) -> {//untested
            prev
                    .lineToSplineHeading(new Pose2d(-72, -50, Math.toRadians(0)))
                    .splineToSplineHeading(new Pose2d(-48, -57, Math.toRadians(0)), Math.toRadians(0))
                    .setConstraints(
                            (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                            (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                    )
                    .lineTo(new Vector2d(-36,-57))
                    .resetConstraints()
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
                    .splineTo(finalPlacePos2Left, Math.toRadians(0))//final place pos 2*******************
                    .addTemporalMarkerOffset(-2, () -> {
                        intake.setIntakePower(0, 0);
                        intake.setIntakeServoPower(0);
                        outtake.createPresetThread(Constants.Slides.superLow, Constants.Arm.placePos, 5, Constants.Extendo.half, true);
                    })
                    .addTemporalMarkerOffset(0.2, () -> {
                        outtake.holdClaw(false);
                        outtake.extend(false);
                    })
                    .setReversed(false)
                    .addTemporalMarkerOffset(0.05, () -> {
                        intake.setIntakePower(0, 0);
                        intake.setIntakeServoPower(0);
                        outtake.createPresetThread(5, Constants.Arm.intakePos, 3, false, false);
                    });
        };
        SequenceFunction WallStackTo3rdPlacePos = (prev) -> {//untested
            prev
                    .lineToSplineHeading(new Pose2d(-72, -50, Math.toRadians(180)))
                    .splineToSplineHeading(new Pose2d(-60, -57, Math.toRadians(180)), Math.toRadians(0))
                    .setConstraints(
                            (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                            (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                    )
                    .lineTo(new Vector2d(50,-50))
                    .addTemporalMarkerOffset(-2, () -> {
                        outtake.holdClaw(true);
                    })
                    .resetConstraints()
                    .splineToLinearHeading(new Pose2d(60,-36, Math.toRadians(215)), Math.toRadians(45))
                    .waitSeconds(0.5)
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
                            telemetry.addData("did see one", optionalPose.isPresent());
                            telemetry.update();
                        }
                    })
                    .splineTo(finalPlacePos3Left, Math.toRadians(0))//final place pos 3*******************
                    .addTemporalMarkerOffset(-2, () -> {
                        intake.setIntakePower(0, 0);
                        intake.setIntakeServoPower(0);
                        outtake.createPresetThread(Constants.Slides.superLow+300, Constants.Arm.placePos, 5, Constants.Extendo.half, true);
                    })
                    .addTemporalMarkerOffset(0.2, () -> {
                        outtake.holdClaw(false);
                        outtake.extend(false);
                    })
                    .setReversed(false)
                    .addTemporalMarkerOffset(0.05, () -> {
                        intake.setIntakePower(0, 0);
                        intake.setIntakeServoPower(0);
                        outtake.createPresetThread(5, Constants.Arm.intakePos, 3, false, false);
                    });
        };
        SequenceFunction CenterStackTo1stPlacePos = (prev) -> {
            prev
                    .splineTo(new Vector2d(0, -12), Math.toRadians(0))
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

                    .splineTo(finalPlacePos1Center, Math.toRadians(0))//final place pos 1*******************
                    .addTemporalMarkerOffset(-2, () -> {
                        intake.setIntakePower(0, 0);
                        intake.setIntakeServoPower(0);
                        outtake.createPresetThread(Constants.Slides.superLow, Constants.Arm.placePos, 5, Constants.Extendo.half, true);
                    })
                    .addTemporalMarkerOffset(0.2, () -> {
                        outtake.holdClaw(false);
                        outtake.extend(false);
                    })
                    .setReversed(false)
                    .addTemporalMarkerOffset(0.05, () -> {
                        intake.setIntakePower(0, 0);
                        intake.setIntakeServoPower(0);
                        outtake.createPresetThread(5, Constants.Arm.intakePos, 3, false, false);
                    });
        };
        //2*****************************************************************************************
        SequenceFunction CenterStackTo2ndPlacePos = (prev) -> {
            prev
                    .splineTo(new Vector2d(0, -12), Math.toRadians(0))
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

                    .splineTo(finalPlacePos2Center, Math.toRadians(0))//final place pos 2*******************
                    .addTemporalMarkerOffset(-2, () -> {
                        intake.setIntakePower(0, 0);
                        intake.setIntakeServoPower(0);
                        outtake.createPresetThread(Constants.Slides.superLow, Constants.Arm.placePos, 1, Constants.Extendo.half, true);
                    })
                    .addTemporalMarkerOffset(0.05, () -> {
                        outtake.holdClaw(false);
                        outtake.extend(false);
                    })
                    .setReversed(false)
                    .addTemporalMarkerOffset(0.2, () -> {
                        intake.setIntakePower(0, 0);
                        intake.setIntakeServoPower(0);
                        outtake.createPresetThread(5, Constants.Arm.intakePos, 3, false, false);
                    });
        };
        //3*****************************************************************************************
        SequenceFunction CenterStackTo3rdPlacePos = (prev) -> {
            prev
                    .splineTo(new Vector2d(0, -12), Math.toRadians(0))
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

                    .splineTo(finalPlacePos3Center, Math.toRadians(0))//final place pos 3*******************
                    .addTemporalMarkerOffset(-2, () -> {
                        intake.setIntakePower(0, 0);
                        intake.setIntakeServoPower(0);
                        outtake.createPresetThread(Constants.Slides.superLow, Constants.Arm.placePos, 5, Constants.Extendo.half, true);
                    })
                    .addTemporalMarkerOffset(0.05, () -> {
                        outtake.holdClaw(false);
                        outtake.extend(false);
                    })
                    .setReversed(false)
                    .addTemporalMarkerOffset(0.2, () -> {
                        intake.setIntakePower(0, 0);
                        intake.setIntakeServoPower(0);
                        outtake.createPresetThread(5, Constants.Arm.intakePos, 3, false, false);
                    });
        };
        SequenceFunction CenterStackToCenterBD = (prev) -> {
            prev
                    .splineTo(new Vector2d(0, -14), Math.toRadians(0))
                    .splineTo(new Vector2d(24, -14), Math.toRadians(0))
                    .splineTo(centerBDPlacePos, Math.toRadians(0))
                    .addTemporalMarkerOffset(-2.4, () -> {
                        specialIntake.setIntakeServo(Constants.SpecialIntake.up);
                        intake.setIntakePower(0, 0);
                        intake.setIntakeServoPower(0);
                        outtake.createPresetThread(Constants.Slides.high, Constants.Arm.placePos, 1, Constants.Extendo.half, true);
                    })
                    .addTemporalMarkerOffset(0.2, () -> {
                        outtake.holdClaw(false);
                        outtake.extend(false);
                    })
                    .waitSeconds(0.1)
                    .strafeLeft(10)
                    .addTemporalMarkerOffset(-0.5, () -> {
                        intake.setIntakePower(0, 0);
                        intake.setIntakeServoPower(0);
                        outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false, false);
                    });
        };
    }
}
