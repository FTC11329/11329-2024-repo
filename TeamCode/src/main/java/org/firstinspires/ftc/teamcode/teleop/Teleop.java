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
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetector;
import org.firstinspires.ftc.teamcode.vision.AprilTagIntoPower;

@TeleOp(name = "Tele-op", group = "Allen op mode")
public class Teleop extends OpMode {
    Claw claw;
    Intake intake;
    Climber climber;
    Outtake outtake;
    Drivetrain drivetrain;
    Cameras cameras;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        claw = new Claw(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        climber = new Climber(hardwareMap);
        cameras = new Cameras(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        boolean fastDriveSpeed = gamepad1.right_bumper;
        double driveForward = -gamepad1.left_stick_y;
        double driveStrafe = -gamepad1.left_stick_x;
        double driveTurn = gamepad1.right_stick_x;

        boolean intakeBool = gamepad1.y;
        boolean outtakeBool = gamepad1.b;

        double slidePower = gamepad1.right_stick_y;

        double armPower = gamepad1.right_trigger - gamepad1.left_trigger;

        double climberPower = gamepad1.right_stick_y * 0;

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
            intake.setIntakePower(Constants.Intake.intake);
        } else if (outtakeBool) {
            claw.setPower(Constants.Claw.outake);
        } else if (gamepad1.x) {
            intake.setIntakePower(Constants.Intake.outake);
        } else {
            claw.setPower(0);
            intake.setIntakePower(0);
        }

        //SLIDES
        outtake.manualSlides(slidePower);
        telemetry.addData("Slide Position", outtake.getSlidePosition());
        telemetry.addData("Slide Target Position", outtake.getSlideTargetPosition());

        //ARM
        outtake.manualArm(armPower);
        telemetry.addData("Arm Position", outtake.getArmPosition());
        telemetry.addData("Sad arm", outtake.arm.isArmScared(outtake.getSlidePosition()));

        //CLIMBER
        climber.setPower(climberPower);

        //PRE-SETS
        if (gamepad1.dpad_up) {
            //high
            outtake.preset(Constants.Slides.highSlides, Constants.Arm.weirdPlacePos);
        }
        if (gamepad1.dpad_right) {
            //med
            outtake.preset(Constants.Slides.medSlides, Constants.Arm.placePos);
        }
        if (gamepad1.dpad_down) {
            //low
            outtake.preset(Constants.Slides.lowSlides, Constants.Arm.placePos);
        }
        if (gamepad1.dpad_left) {
            //intake
            outtake.preset(0, 0.01);
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
