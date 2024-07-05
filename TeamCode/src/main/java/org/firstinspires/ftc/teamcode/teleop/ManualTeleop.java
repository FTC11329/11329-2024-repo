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

@TeleOp(name = "Manual Demo (don't run)", group = "z not Allen op mode")
public class ManualTeleop extends OpMode {
    ElapsedTime elapsedTime = new ElapsedTime();
    boolean extended = false;
    boolean extendDebounce = false;
    boolean grabbedFront = false;
    boolean grabbedBack = false;
    boolean grabFrontDebounce = false;
    boolean grabBackDebounce = false;
    boolean isClimberUp = false;

    double slidePower = 0;
    double armPower = 0;
    double wristTime = 0;

    int climberPos = 0;


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
    }

    @Override
    public void loop() {
        //CLIMBER
        double climberPower = gamepad2.right_stick_y;
        boolean climberDownBool = gamepad2.dpad_left;
        boolean climberUpBool = gamepad2.left_stick_button;
        boolean climberFireBool = gamepad2.right_stick_button;
        if (climberUpBool) {
            climberPos = Constants.Climber.climb;
            isClimberUp = true;

        } else if (climberFireBool) {
            climberPos = Constants.Climber.climberFire;
            isClimberUp = true;

        } else if (climberDownBool) {
            climberPos = Constants.Climber.down;
        }
        climberPos += climberPower * Constants.Climber.manualClimberPower;
        if (gamepad2.back) {
            plane.fire();
        } else {
            plane.hold();
        }

        //SPECIAL INTAKE
        specialIntake.manualHeight(gamepad2.right_stick_x);

        //INTAKE
        boolean intakeBool = gamepad2.y || gamepad1.right_stick_button;
        boolean intakeOuttakeBool = gamepad2.x;

        if (intakeBool) {
            outtake.presetSlides(Constants.Slides.whileIntaking);
        }
        if (intakeBool && !outtake.isFull()) {
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

        //SLIDES
        slidePower = (gamepad2.right_trigger - gamepad2.left_trigger);
        telemetry.addData("Slide Position", outtake.getSlidePosition());
        telemetry.addData("Slide Target Position", outtake.getSlideTargetPosition());

        //ARM
        armPower = gamepad2.left_stick_y;

        //CLAW
        double wristClawPower = gamepad2.left_stick_x;
        boolean grabBoth = gamepad2.b;
        boolean grabFront = gamepad2.right_bumper || grabBoth;
        boolean grabBack = gamepad2.left_bumper || grabBoth;
        if (wristClawPower < -0.5 && elapsedTime.milliseconds() > (Constants.Claw.msChange + wristTime)) {
            wristTime = elapsedTime.milliseconds();
            outtake.manualWrist(1);
        } else if (wristClawPower >  0.5 && elapsedTime.milliseconds() > (Constants.Claw.msChange + wristTime)) {
            wristTime = elapsedTime.milliseconds();
            outtake.manualWrist(-1);
        } else {
            wristTime = -100;
        }

        //EXTENDO
        if (gamepad2.a && extendDebounce) {
            extended = !extended;
            extendDebounce = false;
        }
        if (!gamepad2.a) {
            extendDebounce = true;
        }

        //for grabbing and dropping pixels
        if (grabFront && grabFrontDebounce) {
            grabbedFront = !grabbedFront;
            grabFrontDebounce = false;
        }
        if (!grabFront) {
            grabFrontDebounce = true;
        }

        if (grabBack && grabBackDebounce) {
            grabbedBack = !grabbedBack;
            grabBackDebounce = false;
        }
        if (!grabBack) {
            grabBackDebounce = true;
        }

        //SETTING VALUES
        climber.setPos(climberPos);

        outtake.manualSlides(slidePower, false);
        outtake.manualArm(armPower);
        if (isClimberUp) {
            outtake.setExtendo(Constants.Extendo.clearClimber);
        } else {
            outtake.extend(extended);
        }

        outtake.holdFrontClaw(grabbedFront);
        outtake.holdBackClaw (grabbedBack) ;
    }

    @Override
    public void stop() {
        intake.stopIntake();
        outtake.stopOuttake();
        drivetrain.stopDrive();
    }
}
