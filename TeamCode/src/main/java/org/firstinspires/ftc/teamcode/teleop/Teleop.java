package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoServo;
import org.firstinspires.ftc.teamcode.subsystems.BackDistanceSensors;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Plane;
import org.firstinspires.ftc.teamcode.subsystems.SpecialIntake;

@TeleOp(name = "Tele-op", group = "Allen op mode")
public class Teleop extends OpMode {
    ElapsedTime elapsedTime = new ElapsedTime();
    double startClimb;
    boolean climbed = false;
    boolean climberDebounce = false;
    int intakeLevel = 6;
    boolean intakeDebounce = false;
    int climberPos = 0;
    double SIntakeStart = 0;
    boolean SIntakeDebounce = true;
    double wristTime = 0;
    boolean presetThreadDebounce = true;
    boolean goingPreset = false;
    boolean atPreset = false;
    boolean backClawDropped = true;
    boolean frontClawDropped = true;
    boolean isClimberUp = false;
    double temp = 0;
    boolean isDroneing = false;
    boolean flipDebounce = false;
    boolean endgameToggle = false;
    boolean endgameDebounce = false;
    boolean goingToClaw = false;
    double backLeft;
    double backRight;
    double backOffset;

    Plane plane;
    Lights lights;
    Intake intake;
    Climber climber;
    volatile Outtake outtake;
    Drivetrain drivetrain;
    SpecialIntake specialIntake;
    DistanceSensors distanceSensors;
    BackDistanceSensors backDistanceSensors;

    @Override
    public void init() {
        plane = new Plane(hardwareMap);
        intake = new Intake(hardwareMap);
        lights = new Lights(hardwareMap);
        climber = new Climber(hardwareMap);
        outtake = new Outtake(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        specialIntake = new SpecialIntake(hardwareMap);
        distanceSensors = new DistanceSensors(hardwareMap);
        backDistanceSensors = new BackDistanceSensors(hardwareMap);

        outtake.holdClaw(false);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        outtake.presetArm(Constants.Arm.intakePos);
    }

    @Override
    public void loop() {
        //INPUTS
        boolean fastDriveSpeed = gamepad1.right_bumper;
        double driveForward = -gamepad1.left_stick_y;
        double driveStrafe = -gamepad1.left_stick_x;
        double driveTurn = -gamepad1.right_stick_x;
        boolean slowStrafeLeft = gamepad1.dpad_left;
        boolean slowStrafeRight = gamepad1.dpad_right;
        boolean align = gamepad1.b;

        boolean intakeBool = gamepad2.y || gamepad1.right_stick_button;
        boolean intakeOuttakeBool = gamepad2.x;

        boolean SIntakeUp = gamepad2.left_bumper && !atPreset;
        boolean SIntakeDown = gamepad2.right_bumper && !atPreset;

        boolean overwriteBool = gamepad1.back;
        double slidePower = gamepad2.right_trigger - gamepad2.left_trigger;
        double slowSlidePower = gamepad1.right_trigger - gamepad1.left_trigger;

        boolean armFix = gamepad1.a;
        double armPower = gamepad2.left_stick_y;

        double wristClawPower = gamepad2.left_stick_x;
        boolean flippingButton = gamepad2.a;
        boolean grabIntake = gamepad2.b;
        boolean dropFrontClaw = gamepad2.right_bumper;
        boolean dropBackClaw = gamepad2.left_bumper;
        boolean dropClaw = gamepad2.b || gamepad1.left_bumper;

        double climberPower = gamepad2.right_stick_y;
        boolean climberDownBool = gamepad2.dpad_left;
        boolean climberUpBool = gamepad2.left_stick_button;
        boolean climberFireBool = gamepad2.right_stick_button;

        boolean planeFire = gamepad1.y || gamepad2.back;

        boolean highPresetBool = gamepad2.dpad_up;
        boolean medPresetBool = gamepad2.dpad_right || gamepad1.dpad_up;
        boolean lowPresetBool = gamepad2.dpad_down;
        boolean intakePresetBool = gamepad2.dpad_left || gamepad1.dpad_down;

        boolean endgameButton = gamepad1.x;

        //DRIVETRAIN
        DriveSpeedEnum driveSpeed;
        if (fastDriveSpeed) {
            driveSpeed = DriveSpeedEnum.Fast;
        } else {
            driveSpeed = DriveSpeedEnum.Slow;
        }
        if (slowStrafeLeft) {
            driveStrafe -= Constants.Drivetrain.fixSpeedStrafe;
        } else if (slowStrafeRight) {
            driveStrafe += Constants.Drivetrain.fixSpeedStrafe;
        }

        if (align) {
            backLeft = backDistanceSensors.getBLeftState();
            backRight = backDistanceSensors.getBRightState();
            backOffset = backRight - backLeft;
            //movement
            if (Math.min(backLeft, backRight) > 11.5) {
                driveForward -= 0.5;
            } else if (Math.max(backLeft, backRight) < 11) {
                driveForward += 0.5;
            }
            //starfing
            else if (backOffset > 7) {
                driveStrafe += 0.75;
            } else if (backOffset < -7) {
                driveStrafe -= 0.75;
            }
            //turning
            else if (backOffset > 0.6) {
                driveTurn -= 0.4;
            } else if (backOffset < -0.6) {
                driveTurn += 0.4;
            }


        }
        drivetrain.drive(driveForward, driveStrafe, driveTurn, driveSpeed);


        //INTAKE
        if (intakeBool && !atPreset) {
            outtake.presetSlides(Constants.Slides.whileIntaking);
        }
        if ((intakeBool || intakeOuttakeBool) && atPreset) {
            intake.setIntakePower(Constants.Intake.intake * 0.5, 0);
            intake.setIntakeServoPower(-0.05);

        } else if (intakeBool && !outtake.isFull()) {
            intake.setIntakePower(Constants.Intake.intake, outtake.getSlideTargetPosition());
            intake.setIntakeServoPower(Constants.Intake.intakeServoIntake);

        } else if (intakeBool && outtake.isFull()) {
            intake.setIntakePower(Constants.Intake.outake, outtake.getSlideTargetPosition());
            intake.setIntakeServoPower(Constants.Intake.intakeServoIntake);

        } else if (intakeOuttakeBool && outtake.isFull()) {
            intake.setIntakePower(Constants.Intake.intake, outtake.getSlideTargetPosition());
            intake.setIntakeServoPower(Constants.Intake.intakeServoIntake);

        } else if (intakeOuttakeBool && !outtake.isFull()) {
            intake.setIntakePower(Constants.Intake.outake, outtake.getSlideTargetPosition());
            intake.setIntakeServoPower(Constants.Intake.intakeServoIntake);

        } else {
            intake.setIntakePower(0, outtake.getSlideTargetPosition());
            intake.setIntakeServoPower(0);
        }

        //SPECIAL INTAKE
        if (SIntakeUp && SIntakeDebounce) {
            intakeLevel = 6;
            SIntakeStart = elapsedTime.milliseconds();
            SIntakeDebounce = false;
        }
        if (SIntakeUp && elapsedTime.milliseconds() - SIntakeStart > 150) {
            intakeLevel = 3;
        }
        if (!SIntakeUp) {
            SIntakeDebounce = true;
        }
        if (SIntakeDown && !intakeDebounce) {
            intakeLevel--;
            intakeDebounce = true;
        } else if (!SIntakeDown && intakeDebounce) {
            intakeDebounce = false;
        }

        if (intakeLevel == 6) {
            specialIntake.setIntakeServo(Constants.SpecialIntake.up);

        } else if  (intakeLevel == 5) {
            specialIntake.setIntakeServo(Constants.SpecialIntake.down5);

        } else if  (intakeLevel == 4) {
            specialIntake.setIntakeServo(Constants.SpecialIntake.down4);

        } else if  (intakeLevel == 3) {
            specialIntake.setIntakeServo(Constants.SpecialIntake.down3);

        } else if  (intakeLevel == 2) {
            specialIntake.setIntakeServo(Constants.SpecialIntake.down2);

        } else if  (intakeLevel == 1) {
            specialIntake.setIntakeServo(Constants.SpecialIntake.down1);
        }

        //SLIDES
        slidePower = slidePower + (slowSlidePower * Constants.Slides.slowManualSlidePower);
        telemetry.addData("Slide Position", outtake.getSlidePosition());
        telemetry.addData("Slide Target Position", outtake.getSlideTargetPosition());

        //ARM
        outtake.manualArm(armPower);
        if (armFix) {
            outtake.presetArm(Constants.Arm.placePos);
            outtake.presetSlides(Constants.Slides.low);
            outtake.holdFrontClaw(true);
            outtake.holdFrontClaw(false);
        }
        telemetry.addData("Arm Position", outtake.getArmPosition());

        //CLAW
        if (wristClawPower < -0.5 && elapsedTime.milliseconds() > (Constants.Claw.msChange + wristTime)) {
            wristTime = elapsedTime.milliseconds();
            outtake.manualWrist(1);
        } else if (wristClawPower >  0.5 && elapsedTime.milliseconds() > (Constants.Claw.msChange + wristTime)) {
            wristTime = elapsedTime.milliseconds();
            outtake.manualWrist(-1);
        } else {
            wristTime = -100;
        }
        //for flipping 180
        if (flippingButton && flipDebounce) {
            if ((outtake.getTriedWristPos() - 4) >= 1) {
                outtake.setWristPos(outtake.getTriedWristPos() - 4);
            } else if ((outtake.getTriedWristPos() + 4) <= 7) {
                outtake.setWristPos(outtake.getTriedWristPos() + 4);
            } else {
                outtake.setWristPos(1);
            }
            flipDebounce = false;
        }
        if (!flipDebounce && !flippingButton) {
            flipDebounce = true;
        }

        //for dropping pixels
        if (!outtake.isEmpty() && grabIntake && !atPreset) {
            goingToClaw = true;
            frontClawDropped = false;
            backClawDropped = false;
            slidePower = -2;
        } else if (outtake.isEmpty() && !atPreset && (outtake.getSlidePosition() < 50)) {
            outtake.holdClaw(false);
        }
        if (goingToClaw && outtake.getSlidePosition() <= 0) {
            goingToClaw = false;
            if (outtake.isFull()) {
                outtake.holdClaw(true);
            } else {
                outtake.holdBackClaw(true);
            }
        }
        if (atPreset) {
            if (dropClaw) {
                outtake.holdClaw(false);
                outtake.extend(false);
                backClawDropped = true;
                frontClawDropped = true;

            } else if (dropFrontClaw && !frontClawDropped) {
                outtake.holdFrontClaw(false);
                frontClawDropped = true;
                if (backClawDropped) {
                    outtake.extend(false);
                }

            } else if (dropBackClaw && !backClawDropped) {
                outtake.holdBackClaw(false);
                backClawDropped = true;
                if (frontClawDropped) {
                    outtake.extend(false);
                }
            }
        }

        //CLIMBER
        if (climberUpBool && !climberDebounce && !climbed) {
            startClimb = elapsedTime.milliseconds();
            climberDebounce = true;
            intakeLevel = 5;

        } else if (!climberUpBool && climbed) {
            climberPos = Constants.Climber.climbButSlightlyDown;
            climbed = false;

        } else if (climberFireBool) {
            climberPos = Constants.Climber.climberFire;
            isDroneing = true;

        } else if (climberDownBool){
            climberPos = Constants.Climber.down;
            intakeLevel = 6;
        }
        if (isClimberUp) {
            outtake.extendo.setExtendo(Constants.Extendo.clearClimber);
        }
        climberPos += climberPower * Constants.Climber.manualClimberPower;
        climber.setPos(climberPos);

        //makes you have to hold the button in to make the climber go up
        if (climberUpBool && (elapsedTime.milliseconds() - startClimb > 300)) {
            climberPos = Constants.Climber.climb;
            climbed = true;
            isClimberUp = true;
        }
        if (isDroneing) {
            outtake.extendo.setExtendo(Constants.Extendo.clearClimber);
            if (climber.getPosition() < 10) {
                isDroneing = false;
            }
            if (intakePresetBool) {
                climberPos = Constants.Climber.down;
            }
        }

        if (!climberUpBool) {
            climberDebounce = false;
        }

        //PLANE
        if (planeFire) {
            plane.fire();
        } else {
            plane.hold();
        }

        //PRE-SETS
        goingPreset = highPresetBool || medPresetBool || lowPresetBool || intakePresetBool;
        if (goingPreset && presetThreadDebounce && !isClimberUp && !endgameToggle) {
            if (highPresetBool && !isDroneing) {
                intakeLevel = 6;
                atPreset = true;
                if (outtake.isBackSensor() && !outtake.isFull()) {
                    outtake.setWristPos(7);
                }
                outtake.createPresetThread(Constants.Slides.high, Constants.Arm.placePos, outtake.getTriedWristPos(), true, !backClawDropped, true);
                if(outtake.isFull()) {
                    frontClawDropped = false;
                    backClawDropped = false;
                } else if (outtake.isBackSensor()) {
                    backClawDropped = false;
                }

            } else if (medPresetBool && !isDroneing) {
                intakeLevel = 6;
                atPreset = true;
                if (outtake.isBackSensor() && !outtake.isFull()) {
                    outtake.setWristPos(7);
                }
                outtake.createPresetThread(Constants.Slides.med, Constants.Arm.placePos, outtake.getTriedWristPos(), true, !backClawDropped, true);
                if(outtake.isFull()) {
                    frontClawDropped = false;
                    backClawDropped = false;
                } else if (outtake.isBackSensor()) {
                    backClawDropped = false;
                }

            } else if (lowPresetBool && !isDroneing) {
                intakeLevel = 6;
                atPreset = true;
                if (outtake.isBackSensor() && !outtake.isFull()) {
                    outtake.setWristPos(7);
                }
                outtake.createPresetThread(Constants.Slides.low, Constants.Arm.placePos, outtake.getTriedWristPos(), true, !backClawDropped, true);
                if(outtake.isFull()) {
                    frontClawDropped = false;
                    backClawDropped = false;
                } else if (outtake.isBackSensor()) {
                    backClawDropped = false;
                }

            } else if (intakePresetBool && !isDroneing) {
                outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false, false);
                intakeLevel = 6;
                atPreset = false;
                backClawDropped = true;
                frontClawDropped = true;
            }
            presetThreadDebounce = false;
        }


        //Endgame
        if (goingPreset && endgameToggle) {
            if (intakePresetBool && !isDroneing) {
                outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false, false);
                intakeLevel = 6;
                atPreset = false;
            } else {
                outtake.createPresetThread(300, Constants.Arm.autoArmDrop, 3, true, true);
                if(outtake.isFull()) {
                    frontClawDropped = false;
                    backClawDropped = false;
                } else if (outtake.isBackSensor()) {
                    backClawDropped = false;
                }
                atPreset = true;
            }
        }
        if (!goingPreset) {
            presetThreadDebounce = true;
        }

        //LIGHTS
        if (outtake.isFull()) {
            lights.setDumbLed(1);
        } else if (!outtake.isEmpty()) {
            lights.setDumbFlash(0.2);
        } else {
            lights.setDumbWave(1,0, 1);
        }

        //FINALE
        if (endgameButton && endgameDebounce) {
            endgameToggle = !endgameToggle;
            endgameDebounce = false;
        }
        if (!endgameButton) {
            endgameDebounce = true;
        }

        outtake.periodic();
        outtake.manualSlides(slidePower, overwriteBool);


        //TEMPORARY
//        telemetry.addData("Slide motor amps", outtake.slides.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Wrist Position", outtake.getTriedWristPos());

        telemetry.addData("climber pos", climberPos);


        telemetry.addData("BL", backLeft);
        telemetry.addData("BR", backRight);

        telemetry.addData("Of", backOffset);

    }

    @Override
    public void stop() {
        intake.stopIntake();
        outtake.stopOuttake();
        drivetrain.stopDrive();
    }
}
