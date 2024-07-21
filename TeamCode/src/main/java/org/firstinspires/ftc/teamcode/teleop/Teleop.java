package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
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
    boolean goingToPreset = false;
    boolean atPreset = false;
    boolean backClawDropped = true;
    boolean frontClawDropped = true;
    boolean isClimberUp = false;
    boolean isDroneing = false;
    boolean flipDebounce = false;
    boolean endgameToggle = false;
    boolean endgameDebounce = false;
    boolean goingToClaw = false;
    double backLeft;
    double backRight;
    double backOffset;
    double intakeDroneTime = 2140000000;
    boolean droneOnce = true;
    boolean lastIntake = false;
    boolean isDroneingDe = false;

    DriveSpeedEnum driveSpeed;


    Plane plane;
    volatile Lights lights;
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

        boolean intakeBool = (gamepad2.y || gamepad1.right_stick_button) && outtake.getSlidePosition() < Constants.Slides.safeSlidePos;
        boolean intakeOuttakeBool = gamepad2.x;

        boolean SIntakeUp = gamepad2.left_bumper && !goingToPreset;
        boolean SIntakeDown = gamepad2.right_bumper && !goingToPreset;

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
        boolean climberDownBool = gamepad2.dpad_left || gamepad2.a;
        boolean climberUpBool = gamepad2.left_stick_button;
        boolean climberFireBool = gamepad2.right_stick_button;

        boolean planeFire = gamepad1.y || gamepad2.back;

        boolean highPresetBool = gamepad2.dpad_up;
        boolean medPresetBool = gamepad2.dpad_right || gamepad1.dpad_up;
        boolean lowPresetBool = gamepad2.dpad_down;
        boolean intakePresetBool = gamepad2.dpad_left || gamepad1.dpad_down;

        boolean endgameButton = gamepad1.x;

        //DRIVETRAIN
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
            if (Math.min(backLeft, backRight) > 9.5) {
                driveForward -= 0.5;
            } else if (Math.max(backLeft, backRight) < 8.5) {
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
        if (intakeBool && !goingToPreset) {
            outtake.presetSlides(Constants.Slides.whileIntaking);
            outtake.setExtendo(Constants.Extendo.whileIntaking);
        }
        if (intakeOuttakeBool && goingToPreset) {
            intake.setIntakePower(Constants.Intake.outake, outtake.getSlideTargetPosition());
            intake.setIntakeServoPower(Constants.Intake.intakeServoIntake);

        } else if (intakeBool && !outtake.isFull() && !goingToPreset) {
            intake.setIntakePower(Constants.Intake.intake, outtake.getSlideTargetPosition());
            intake.setIntakeServoPower(Constants.Intake.intakeServoIntake);

        } else if (intakeBool && outtake.isFull() && !goingToPreset) {
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

        if (!intakeBool && lastIntake && !goingToPreset) {
            outtake.extend(false);
            outtake.presetSlides(0);
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

        //ARM
        outtake.manualArm(armPower);
        if (armFix) {
            outtake.createPresetThread(Constants.Slides.superLow, Constants.Arm.fixPos, 3, true, true, true);
            goingToPreset = true;
            outtake.holdFrontClaw(true);
            outtake.holdBackClaw(false);
            frontClawDropped = false;
            backClawDropped = true;
        }

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
        //for preset of claw
        if (gamepad2.y) {
            outtake.setWristPos(3);
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
        if (!outtake.isEmpty() && grabIntake && !goingToPreset) {
            goingToClaw = true;
            frontClawDropped = false;
            backClawDropped = false;
            slidePower = -2;
            outtake.extend(false);
        } else if (outtake.isEmpty() && !goingToPreset && (outtake.getSlidePosition() < 50)) {
            outtake.holdClaw(false);
        }
        if (goingToClaw && outtake.getSlidePosition() <= 0) {
            goingToClaw = false;
            outtake.holdClaw(true);
        }
        if (goingToPreset) {
            if (dropClaw) {
                outtake.holdClaw(false);
                outtake.extend(false);
                backClawDropped = true;
                frontClawDropped = true;
                goingToPreset = false;

            } else if (dropFrontClaw && !frontClawDropped) {
                outtake.holdFrontClaw(false);
                frontClawDropped = true;
                if (backClawDropped) {
                    outtake.extend(false);
                    goingToPreset = false;
                }

            } else if (dropBackClaw && !backClawDropped) {
                outtake.holdBackClaw(false);
                backClawDropped = true;
                if (frontClawDropped) {
                    outtake.extend(false);
                    goingToPreset = false;
                }
            }
        }
        //PLANE
        if (planeFire) {
            plane.fire();
            intakeDroneTime = elapsedTime.milliseconds() + 250;
        } else {
            plane.hold();
        }
        if (elapsedTime.milliseconds() > intakeDroneTime && droneOnce) {
            outtake.holdClaw(false);
            outtake.extend(true);
            climberDownBool = true;
            droneOnce = false;
        }
        if (elapsedTime.milliseconds() > intakeDroneTime + 150) {
            intakePresetBool = true;
            droneOnce = true;
            intakeDroneTime = 2140000000;
        }

        //CLIMBER
        if (climberUpBool && !climberDebounce && !climbed) {
            startClimb = elapsedTime.milliseconds();
            climberDebounce = true;
            intakeLevel = 5;

        } else if (climbed && climber.getPosition() < -1700) {
            climberPos = Constants.Climber.climbButSlightlyDown;
            climbed = false;

        } else if (climberFireBool && isDroneingDe) {
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
        if (climberUpBool && (elapsedTime.milliseconds() - startClimb > 100)) {
            climberPos = Constants.Climber.climb;
            climbed = true;
            isClimberUp = true;
        }
        if (!climberUpBool) {
            climberDebounce = false;
        }

        if (intakePresetBool && isDroneing) {
            climberPos = Constants.Climber.down;
            isDroneing = false;
        }
        if (isDroneing && isDroneingDe) {
            outtake.extendo.setExtendo(Constants.Extendo.clearClimber);

            if (!climberFireBool) {
                climberPos = Constants.Climber.climberFireFar;
            } else {
                climberPos = Constants.Climber.climberFireClose;
            }
            isDroneingDe = false;
        }
        if (!climberFireBool) {
            isDroneingDe = true;
        }

        //PRE-SETS
        goingPreset = highPresetBool || medPresetBool || lowPresetBool || intakePresetBool;
        if (goingPreset && presetThreadDebounce && !isClimberUp && !endgameToggle && !goingToPreset) {
            if (highPresetBool && !isDroneing) {
                intakeLevel = 6;
                goingToPreset = true;
                if (outtake.isBackSensor() && !outtake.isFull()) {
                    outtake.setWristPos(7);
                }
                outtake.createPresetThread(Constants.Slides.high, Constants.Arm.placePos, 5, true, !backClawDropped, true);
                if(outtake.isFull()) {
                    frontClawDropped = false;
                    backClawDropped = false;
                } else if (outtake.isBackSensor()) {
                    backClawDropped = false;
                }

            } else if (medPresetBool && !isDroneing) {
                intakeLevel = 6;
                goingToPreset = true;
                if (outtake.isBackSensor() && !outtake.isFull()) {
                    outtake.setWristPos(7);
                }
                outtake.createPresetThread(Constants.Slides.med, Constants.Arm.placePos, 5, true, !backClawDropped, true);
                if(outtake.isFull()) {
                    frontClawDropped = false;
                    backClawDropped = false;
                } else if (outtake.isBackSensor()) {
                    backClawDropped = false;
                }

            } else if (lowPresetBool && !isDroneing) {
                intakeLevel = 6;
                goingToPreset = true;
                if (outtake.isBackSensor() && !outtake.isFull()) {
                    outtake.setWristPos(7);
                }
                outtake.createPresetThread(Constants.Slides.low, Constants.Arm.placePos, 5, true, !backClawDropped, true);
                if(outtake.isFull()) {
                    frontClawDropped = false;
                    backClawDropped = false;
                } else if (outtake.isBackSensor()) {
                    backClawDropped = false;
                }

            } else if (intakePresetBool && !isDroneing) {
                outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false, false);
                intakeLevel = 6;
                goingToPreset = false;
                backClawDropped = true;
                frontClawDropped = true;
            }
            presetThreadDebounce = false;


        } else if (goingPreset && presetThreadDebounce && !isClimberUp && !endgameToggle && goingToPreset) {
            if (highPresetBool && !isDroneing) {
                intakeLevel = 6;
                outtake.createPresetThread(Constants.Slides.high, Constants.Arm.placePos, outtake.getTriedWristPos(), true, true, true);

            } else if (medPresetBool && !isDroneing) {
                intakeLevel = 6;
                outtake.createPresetThread(Constants.Slides.med, Constants.Arm.placePos, outtake.getTriedWristPos(), true, true, true);

            } else if (lowPresetBool && !isDroneing) {
                intakeLevel = 6;
                outtake.createPresetThread(Constants.Slides.low, Constants.Arm.placePos, outtake.getTriedWristPos(), true, true, true);

            } else if (intakePresetBool && !isDroneing) {
                outtake.createPresetThread(Constants.Slides.intake, Constants.Arm.intakePos, 3, false, false);
                intakeLevel = 6;
                goingToPreset = false;
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
                goingToPreset = false;
            } else {
                outtake.createPresetThread(300, Constants.Arm.autoArmDrop, 3, true, true);
                if(outtake.isFull()) {
                    frontClawDropped = false;
                    backClawDropped = false;
                } else if (outtake.isBackSensor()) {
                    backClawDropped = false;
                }
                goingToPreset = true;
            }
        }
        if (!goingPreset) {
            presetThreadDebounce = true;
        }

        //LIGHTS
        if (outtake.isFull()) {
            lights.setDumbLed(1);
//            if (!lightDe) {
//                lights.killDumbWaveThread();
//            }
//            lightDe = true;
        } else if (!outtake.isEmpty()) {
            lights.setDumbFlash(0.2);
//            if (!lightDe) {
//                lights.killDumbWaveThread();
//            }
//            lightDe = true;

        } else {
            lights.setDumbWave(1, 0, 1);
//            if (lightDe) {
//                lights.startDumbWaveThread();
//            }
//            lightDe = false;
        }

        //FINALE
        if (endgameButton && endgameDebounce) {
            endgameToggle = !endgameToggle;
            endgameDebounce = false;
        }
        if (!endgameButton) {
            endgameDebounce = true;
        }

        if (goingToPreset && outtake.getSlidePosition() > Constants.Slides.safeSlidePos) {
            atPreset = true;
        } else {
            atPreset = false;
        }

        outtake.periodic(atPreset);
        if (outtake.getSlideTargetPosition() != Constants.Slides.safeSlidePos + 100) {
            outtake.manualSlides(slidePower, overwriteBool);
        }



        lastIntake = intakeBool;
        //telemetry
//        telemetry.addData("Slide Position", outtake.getSlidePosition());
//        telemetry.addData("Slide Target Position", outtake.getSlideTargetPosition());
//        telemetry.addData("Arm Position", outtake.getArmPosition());
//        telemetry.addData("Slide motor amps", outtake.slides.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("Wrist Position", outtake.getTriedWristPos());
//        telemetry.addData("Climber pos", climberPos);
//        telemetry.addData("Climber actual", climber.getPosition());
//        telemetry.addData("F", outtake.clawSensor.getFrontDistance(DistanceUnit.INCH));
//        telemetry.addData("B", outtake.clawSensor.getBackDistance(DistanceUnit.INCH));
//        telemetry.addData("BDL", backDi stanceSensors.getBLeftState());
//        telemetry.addData("BDR", backDistanceSensors.getBRightState());

//        telemetry.addData("drone", isDroneing);
//        telemetry.addData("de", droneClimbDe);
//        telemetry.addData("togg", droneClimbTogg);

        //TEMPORARY
    }

    @Override
    public void stop() {
        intake.stopIntake();
        outtake.stopOuttake();
        drivetrain.stopDrive();
        lights.killDumbWaveThread();
    }
}
