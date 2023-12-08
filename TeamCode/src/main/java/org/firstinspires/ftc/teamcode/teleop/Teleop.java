package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.utility.RunAfterTime.runAfterTime;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Plane;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.utility.AprilTagToRoadRunner;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetector;
import org.firstinspires.ftc.teamcode.vision.AprilTagIntoPower;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Optional;

@TeleOp(name = "Tele-op", group = "Allen op mode")
public class Teleop extends OpMode {
    WebcamName webcam1;

    Claw claw;
//    Plane plane;
    Intake intake;
    Climber climber;
    Outtake outtake;
    Drivetrain drivetrain;

    AprilTagDetector aprilTagDetector;
    AprilTagIntoPower aprilTagIntoPower;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    @Override
    public void init() {
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        telemetry.addData("Status", "Initialized");

        claw = new Claw(hardwareMap);
//         plane = new Plane(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        climber = new Climber(hardwareMap);

        drivetrain = new Drivetrain(hardwareMap, telemetry);

        aprilTagDetector = new AprilTagDetector();
        aprilTagIntoPower = new AprilTagIntoPower();
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline();

//        aprilTagDetectionPipeline.AprilTagInit(webcam1, telemetry);
    }

    @Override
    public void loop() {
        //INPUTS
        boolean fastDriveSpeed = gamepad1.right_bumper;
        double driveForward = -gamepad1.left_stick_y;
        double driveStrafe  = -gamepad1.left_stick_x;
        double driveTurn    =  gamepad1.right_stick_x;

        boolean intakeBool  = gamepad1.y;
        boolean outtakeBool = gamepad1.b;

        double slidePower = gamepad1.right_stick_y;

        double armPower = gamepad1.right_stick_y;

        double climberPower = gamepad1.right_trigger - gamepad1.left_trigger;

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
            intake.setIntakePower(Constants.Claw.intake);
        } else if (outtakeBool) {
            claw.setPower(Constants.Claw.outake);
            intake.setIntakePower(Constants.Intake.outake);
        } else {
            claw.setPower(0);
            intake.setIntakePower(0);
        }

        //SLIDES
        outtake.manualSlides(slidePower);

        //ARM
        outtake.manualArm(armPower);

        //CLIMBER
        climber.setPower(climberPower);

        /*
        //PLANE
        if (gamepad1.dpad_up) {
            plane.setPos(Constants.Plane.release);
        } else {
            plane.setPos(Constants.Plane.hold);
        }

        //PRE-SETS
        if (gamepad1.dpad_right) {
            outtake.presetSlides(Constants.Slides.medSlides);
        }
        */

        outtake.periodic();
    }
    @Override
    public void stop() {
         claw.stopClaw();
         intake.stopIntake();
         outtake.stopOuttake();
         drivetrain.stopDrive();
         aprilTagDetectionPipeline.AprilTagStop();
    }
}
