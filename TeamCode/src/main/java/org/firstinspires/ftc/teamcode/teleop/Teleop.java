package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Cameras;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Plane;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

@TeleOp(name = "Tele-op", group = "Allen op mode")
public class Teleop extends OpMode {
    Claw claw;
    Plane plane;
    Intake intake;
    Slides slides; //temp
    Climber climber;
    Outtake outtake;
    Cameras cameras;
    Drivetrain drivetrain;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        claw = new Claw(hardwareMap);
        plane = new Plane(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        slides = outtake.slides;
        climber = new Climber(hardwareMap);
        cameras = new Cameras(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        //INPUTS
        boolean fastDriveSpeed = gamepad1.right_bumper;
        double driveForward = -gamepad1.left_stick_y;
        double driveStrafe = -gamepad1.left_stick_x;
        double driveTurn = -gamepad1.right_stick_x;

        boolean intakeBool = gamepad2.y;
        boolean clawOuttakeBool = gamepad2.b;
        boolean intakeOuttakeBool = gamepad2.x;

        double slidePower = gamepad2.right_trigger - gamepad1.left_trigger;
        boolean downSlidesBool = gamepad2.left_bumper;
        boolean upSlidesBool = gamepad2.right_bumper;

        double armPower = gamepad2.left_stick_y;

        int climberPower = (int) (gamepad2.right_stick_y * Constants.Climber.manualClimberPower);

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
        } else {
            driveSpeed = DriveSpeedEnum.Slow;
        }
        drivetrain.drive(driveForward, driveStrafe, driveTurn, driveSpeed);

        //INTAKE
        if (intakeBool) {
            claw.setPower(Constants.Claw.intake);
            intake.setIntakePower(Constants.Intake.intake, outtake.getSlideTargetPosition());
        } else if (clawOuttakeBool) {
            claw.setPower(Constants.Claw.outake);
        } else if (intakeOuttakeBool) {
            intake.setIntakePower(Constants.Intake.outake, outtake.getSlideTargetPosition());
        } else {
            claw.setPower(0);
            intake.setIntakePower(0, outtake.getSlideTargetPosition());
        }

        //SLIDES
//        outtake.manualSlides(slidePower);
        if (downSlidesBool || upSlidesBool) {
            outtake.slides.manualPosition(slidePower);
        }
//        outtake.upSlide(Constants.Slides.upAmount, upSlidesBool);
//        outtake.upSlide(-Constants.Slides.upAmount, downSlidesBool);
        telemetry.addData("Slide Position", outtake.getSlidePosition());
        telemetry.addData("Slide Target Position", outtake.getSlideTargetPosition());

        //ARM
        outtake.manualArm(armPower);
        telemetry.addData("Arm Position", outtake.getArmPosition());

        //CLIMBER

        int climberPos =  climberPower + climber.getPosition();
        telemetry.addData("climber power ", climberPower);
        telemetry.addData("climber target pos ", climberPos);
        telemetry.addData("climver pos", climber.getPosition());
        climber.setPos(climberPos);

        //PLANE
        if (planeFire) {
            plane.fire();
        } else {
            plane.hold();
        }

        //PRE-SETS
        if (highPresetBool) {
            outtake.preset(Constants.Slides.high, Constants.Arm.weirdPlacePos);
        }
        if (medPresetBool) {
            outtake.preset(Constants.Slides.med, Constants.Arm.placePos);
        }
        if (lowPresetBool) {
//            outtake.preset(Constants.Slides.low, Constants.Arm.placePos);
            outtake.presetSlides(Constants.Slides.low);
            outtake.presetArm(Constants.Arm.placePos);
        }
        if (intakePresetBool) {
            outtake.preset(Constants.Slides.intake, Constants.Arm.intakePos);
        }

        outtake.periodic();
    }

    @Override
    public void stop() {
        claw.stopClaw();
        intake.stopIntake();
        outtake.stopOuttake();
        drivetrain.stopDrive();
    }
}
