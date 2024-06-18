package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Cameras;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ClawSensor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.SpecialIntake;
import org.firstinspires.ftc.teamcode.utility.SequenceFunction;

import java.util.Optional;

public class ConstantCRIPaths {
    static Pose2d pickupSpecial = new Pose2d(-16.5, -13, Math.toRadians(135));

    public PlacePurplePathsRed placePurplePathsRed;
    public PickupWhitePixelStack pickupWhitePixelStack;
    public PlaceOnBackDrop placeOnBackDrop;

    public ConstantCRIPaths(Telemetry telemetry, Claw claw, Intake intake, Outtake outtake, Cameras cameras, ClawSensor clawSensor, Drivetrain drivetrain, SpecialIntake specialIntake) {
        placePurplePathsRed = new PlacePurplePathsRed(claw, intake, outtake, cameras, clawSensor, drivetrain, specialIntake);
        pickupWhitePixelStack = new PickupWhitePixelStack(claw, intake, outtake, cameras, clawSensor, drivetrain, specialIntake);
        placeOnBackDrop = new PlaceOnBackDrop(telemetry, claw, intake, outtake, cameras, clawSensor, drivetrain, specialIntake);
    }
    @Config
    public class PlacePurplePathsRed {
        private Claw claw;
        private Intake intake;
        private Outtake outtake;
        private Cameras cameras;
        private ClawSensor clawSensor;
        private Drivetrain drivetrain;
        private SpecialIntake specialIntake;
        public PlacePurplePathsRed(Claw claw, Intake intake, Outtake outtake, Cameras cameras, ClawSensor clawSensor, Drivetrain drivetrain, SpecialIntake specialIntake) {
            this.claw = claw;
            this.intake = intake;
            this.outtake = outtake;
            this.cameras = cameras;
            this.clawSensor = clawSensor;
            this.drivetrain = drivetrain;
            this.specialIntake = specialIntake;
        }
            //Left *************************************************************************************************************************************************************************************************************************************

         SequenceFunction LeftPlacePos1 = (prev) -> {
            prev
                    .waitSeconds(1);

        };
        SequenceFunction LeftPlacePos2 = (prev) -> {
            prev
                    .waitSeconds(1);
        };
        SequenceFunction LeftPlacePos3 = (prev) -> {
            prev
                    .waitSeconds(1);
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
                    .lineToLinearHeading(new Pose2d(-12,-30, 90))
                    .lineToLinearHeading(pickupSpecial.plus(new Pose2d(4.5,0, 0)));
        };
        SequenceFunction CenterPlacePos2 = (prev) -> {
            prev
                    .setConstraints(
                            (displacement, pose, derivative, baseRobotVelocity) -> 60, //vel
                            (displacement, pose, derivative, baseRobotVelocity) -> 60  //acc
                    )
                    .addTemporalMarker(() -> {
                        outtake.createPresetThread(150, Constants.Arm.autoArmDrop, 3, Constants.Extendo.auto, true);
                    })
                    .lineToLinearHeading(new Pose2d(-12, -15, Math.toRadians(90)))
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
                    .lineToLinearHeading(pickupSpecial.plus(new Pose2d(4.5,0, 0)));
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
                    .splineToLinearHeading(pickupSpecial.plus(new Pose2d(4.5,0, 0)), Math.toRadians(90));
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
        private Claw claw;
        private Intake intake;
        private Outtake outtake;
        private Cameras cameras;
        private ClawSensor clawSensor;
        private Drivetrain drivetrain;
        private SpecialIntake specialIntake;
        public PickupWhitePixelStack(Claw claw, Intake intake, Outtake outtake, Cameras cameras, ClawSensor clawSensor, Drivetrain drivetrain, SpecialIntake specialIntake) {
            this.claw = claw;
            this.intake = intake;
            this.outtake = outtake;
            this.cameras = cameras;
            this.clawSensor = clawSensor;
            this.drivetrain = drivetrain;
            this.specialIntake = specialIntake;
        }
        SequenceFunction CenterStackFromCenter1st = (prev) -> {
            prev
                    .resetConstraints()
                    .setReversed(true)
                    .lineToLinearHeading(pickupSpecial)
                    //1st pickup start
                    .addTemporalMarkerOffset(-0.2, () -> {
                        clawSensor.setRunInAuto(true);
                        outtake.presetSlides(20);
                        intake.setIntakePower(Constants.Intake.intake, 0);
                        intake.setIntakeServoPower(0.75);
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
                    })
                    .addTemporalMarkerOffset(4, () -> {
                        intake.setIntakePower(0, 0);
                    });
        };
    }
    @Config
    public class PlaceOnBackDrop {
        private Telemetry telemetry;
        private Claw claw;
        private Intake intake;
        private Outtake outtake;
        private Cameras cameras;
        private ClawSensor clawSensor;
        private Drivetrain drivetrain;
        private SpecialIntake specialIntake;
        public PlaceOnBackDrop(Telemetry telemetry, Claw claw, Intake intake, Outtake outtake, Cameras cameras, ClawSensor clawSensor, Drivetrain drivetrain, SpecialIntake specialIntake) {
            this.telemetry = telemetry;
            this.claw = claw;
            this.intake = intake;
            this.outtake = outtake;
            this.cameras = cameras;
            this.clawSensor = clawSensor;
            this.drivetrain = drivetrain;
            this.specialIntake = specialIntake;
        }
        SequenceFunction CenterStackTo3rdPos = (prev) -> {
            prev
                    .splineTo(new Vector2d(0, -12), Math.toRadians(0))
                    .splineTo(new Vector2d(40, -12), Math.toRadians(0))
                    .waitSeconds(0.1)
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

                    .splineTo(new Vector2d(72, -38), Math.toRadians(0))//final place pos 1
                    .addTemporalMarkerOffset(-1, () -> {
                        intake.setIntakePower(0, 0);
                        intake.setIntakeServoPower(0);
                        outtake.createPresetThread(Constants.Slides.superLow, Constants.Arm.placePos, 1, Constants.Extendo.half, true);
                    })
                    .addTemporalMarkerOffset(0.2, () -> {
                        outtake.holdClaw(false);
                        outtake.extend(false);
                    })
                    .waitSeconds(0.1)
                    .setReversed(false)
                    .addTemporalMarkerOffset(0.2, () -> {
                        intake.setIntakePower(0, 0);
                        intake.setIntakeServoPower(0);
                        outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false, false);
                    });
        };
    }
}
