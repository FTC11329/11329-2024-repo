package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoServo;
import org.firstinspires.ftc.teamcode.subsystems.Cameras;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ClawSensor;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Plane;
import org.firstinspires.ftc.teamcode.subsystems.SpecialIntake;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Optional;

@TeleOp(name = "Tele-op", group = "Allen op mode")
public class Teleop extends OpMode {
    ElapsedTime elapsedTime = new ElapsedTime();
    double startClimb;
    boolean climbed = false;
    boolean climberDebounce = false;
    int intakeLevel = 6;
    boolean intakeDebounce = false;
    int climberPos = 0;
    double temp1 = 0;
    double photoTime = 0;
    Optional<Pose2d> optionalPose = Optional.empty();

    Claw claw;
    Plane plane;
    Lights lights;
    Intake intake;
    Climber climber;
    Outtake outtake;
    AutoServo autoServo;
    ClawSensor clawSensor;
    Drivetrain drivetrain;
    SpecialIntake specialIntake;
    DistanceSensors distanceSensors;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //VARIABLES

        claw = new Claw(hardwareMap);
        plane = new Plane(hardwareMap);
        intake = new Intake(hardwareMap);
        lights = new Lights(hardwareMap);
        climber = new Climber(hardwareMap);
        outtake = new Outtake(hardwareMap);
        autoServo = new AutoServo(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        clawSensor = new ClawSensor(hardwareMap);
        specialIntake = new SpecialIntake(hardwareMap);
        distanceSensors = new DistanceSensors(hardwareMap);
    }

    @Override
    public void start() {
        outtake.presetArm(Constants.Arm.intakePos);
    }

    @Override
    public void loop() {
        //INPUTS
        boolean superFastSpeed = gamepad1.left_bumper;
        boolean fastDriveSpeed = gamepad1.right_bumper;
        double driveForward = -gamepad1.left_stick_y;
        double driveStrafe = -gamepad1.left_stick_x;
        double driveTurn = -gamepad1.right_stick_x;
        boolean slowStrafeLeft = gamepad1.dpad_left;
        boolean slowStrafeRight = gamepad1.dpad_right;

        boolean intakeBool = gamepad2.y || gamepad1.right_stick_button;
        boolean clawOuttakeBool = gamepad2.b;
        boolean intakeOuttakeBool = gamepad2.x;

        boolean SIntakeUp = gamepad2.left_bumper;
        boolean SIntakeDown = gamepad2.right_bumper;
        boolean overwriteBool = gamepad1.back;
        double slidePower = gamepad2.right_trigger - gamepad2.left_trigger;
        double slowSlidePower = gamepad1.right_trigger - gamepad1.left_trigger;

        double armPower = -gamepad2.left_stick_y;
        boolean armFix = gamepad1.a;

        double climberPower = gamepad2.right_stick_y;
        boolean climberDownBool = gamepad2.a;
        boolean climberUpBool = gamepad2.left_stick_button;
        boolean climberFireBool = gamepad2.right_stick_button;

        boolean planeFire = gamepad2.back;

        boolean highPresetBool = gamepad2.dpad_up;
        boolean medPresetBool = gamepad2.dpad_right;
        boolean lowPresetBool = gamepad2.dpad_down;
        boolean intakePresetBool = gamepad2.dpad_left;

        //DRIVETRAIN
        //Testing wheels
        /*
                if (gamepad1.x){
                    drivetrain.rightFront.setPower(0.25);
                } else {
                    drivetrain.rightFront.setPower(0);
                }

                if (gamepad1.y){
                    drivetrain.leftFront.setPower(0.25);
                } else {
                    drivetrain.leftFront.setPower(0);
                }

                if (gamepad1.b){
                    drivetrain.rightRear.setPower(0.25);
                } else {
                    drivetrain.rightRear.setPower(0);
                }

                if (gamepad1.a){
                    drivetrain.leftRear.setPower(0.25);
                } else {
                    drivetrain.leftRear.setPower(0);
                }
        */
        DriveSpeedEnum driveSpeed;
        if (fastDriveSpeed) {
            driveSpeed = DriveSpeedEnum.Fast;
        } else if (superFastSpeed) {
            driveSpeed = DriveSpeedEnum.SuperFast;
        } else {
            driveSpeed = DriveSpeedEnum.Slow;
        }
        if (slowStrafeLeft) {
            driveStrafe -= Constants.Drivetrain.fixSpeedStrafe;
            driveForward -= Constants.Drivetrain.fixSpeedForward;
        } else if (slowStrafeRight) {
            driveStrafe += Constants.Drivetrain.fixSpeedStrafe;
            driveForward -= Constants.Drivetrain.fixSpeedForward;
        }
        drivetrain.drive(driveForward, driveStrafe, driveTurn, driveSpeed);


        //INTAKE
        if (intakeBool && !clawSensor.isFull()) {
            claw.setPower(Constants.Claw.intake);
            intake.setIntakePower(Constants.Intake.intake, outtake.getSlideTargetPosition());

        } else if (intakeBool && clawSensor.isFull()) {
            intake.setIntakePower(Constants.Intake.outake, outtake.getSlideTargetPosition());

        } else if (clawOuttakeBool) {
            claw.setPower(Constants.Claw.outake);

        } else if (intakeOuttakeBool && clawSensor.isFull()) {
            claw.setPower(Constants.Claw.intake);
            intake.setIntakePower(Constants.Intake.intake, outtake.getSlideTargetPosition());

        } else if (intakeOuttakeBool && !clawSensor.isFull()) {
            intake.setIntakePower(Constants.Intake.outake, outtake.getSlideTargetPosition());

        } else {
            claw.setPower(0);
            intake.setIntakePower(0, outtake.getSlideTargetPosition());
        }

        //SPECIAL INTAKE
        if (SIntakeUp) {
            intakeLevel = 6;
        }

        if (SIntakeDown && !intakeDebounce) {
            intakeLevel --;
            intakeDebounce = true;
        } else if (!SIntakeDown && intakeDebounce){
            intakeDebounce = false;
        }

        switch (intakeLevel) {
            case 6: {
                specialIntake.setIntakeServo(Constants.SpecialIntake.up);
                break;
            }
            case 5: {
                specialIntake.setIntakeServo(Constants.SpecialIntake.down5);
                break;
            }
            case 4: {
                specialIntake.setIntakeServo(Constants.SpecialIntake.down4);
                break;
            }
            case 3: {
                specialIntake.setIntakeServo(Constants.SpecialIntake.down3);
                break;
            }
            case 2: {
                specialIntake.setIntakeServo(Constants.SpecialIntake.down2);
                break;
            }
            case 1: {
                specialIntake.setIntakeServo(Constants.SpecialIntake.down1);
                break;
            }
        }


        //SLIDES
        slidePower = slidePower + (slowSlidePower * Constants.Slides.slowManualSlidePower);
        outtake.manualSlides(slidePower, overwriteBool);
        telemetry.addData("Slide Position", outtake.getSlidePosition());
        telemetry.addData("Slide Target Position", outtake.getSlideTargetPosition());

        //ARM
        outtake.manualArm(armPower);
        if (armFix) {
            outtake.presetArm(Constants.Arm.fixPos);
        }
        telemetry.addData("Arm Position", outtake.getArmPosition());

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
        } else if (climberDownBool){
            climberPos = Constants.Climber.down;
            intakeLevel = 6;
        }
        climberPos += climberPower * Constants.Climber.manualClimberPower;
        climber.setPos(climberPos);
        telemetry.addData("climber pos", climberPos);

        //makes you have to hold the button in to make the climber go up
        if (climberUpBool && (elapsedTime.milliseconds() - startClimb > 300)) {
            climberPos = Constants.Climber.climb;
            climbed = true;
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
        if (highPresetBool) {
            outtake.preset(Constants.Slides.high, Constants.Arm.weirdPlacePos);
            intakeLevel = 6;
        }
        if (medPresetBool) {
            outtake.preset(Constants.Slides.med, Constants.Arm.placePos);
            intakeLevel = 6;
        }
        if (lowPresetBool) {
            outtake.preset(Constants.Slides.low, Constants.Arm.placePos);
            intakeLevel = 6;
        }
        if (intakePresetBool) {
            outtake.preset(Constants.Slides.intake, Constants.Arm.intakePos);
            intakeLevel = 6;
        }

        //LIGHTS
        if (clawSensor.isFull()) {
            lights.setDumbLed(0.5);
        } else if (!clawSensor.isEmpty()) {
            lights.setDumbFlash(0.2);
        } else {
            lights.setDumbWave(1,0, 1);
        }

        //FINALE
        outtake.periodic();

        //TEMPORARY
        //for auto servo
        if (gamepad1.x) {
            autoServo.DropLeft();
        } else if (gamepad1.b) {
            autoServo.DropRight();
        } else {
            autoServo.upBoth();
        }
    }

    @Override
    public void stop() {
        claw.stopClaw();
        intake.stopIntake();
        outtake.stopOuttake();
        drivetrain.stopDrive();
    }
}
