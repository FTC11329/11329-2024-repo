package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Plane;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.utility.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="Tele-op", group="Allen op mode")
public class Teleop extends OpMode
{
    WebcamName webcam1;

    Claw claw;
    Plane plane;
    Outtake outtake;
    Intake intake;
    Drivetrain drivetrain;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private boolean auto1 = false;
    private boolean auto2 = false;

    double intakePower;
    double slidePower;

    @Override
    public void init() {
        intakePower = 0;

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        telemetry.addData("Status", "Initialized");

//        claw = new Claw(hardwareMap);
//        plane = new Plane(hardwareMap);
//        outtake = new Outtake(hardwareMap);
//        intake = new Intake(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline();

        aprilTagDetectionPipeline.AprilTagInit(webcam1, telemetry);
    }

    @Override
    public void start() {}

    @Override
    public void loop() {
        auto1 = gamepad1.a;
        auto2 = gamepad1.b;
        slidePower = gamepad1.left_trigger - gamepad1.right_trigger;
        List<Double> driveList;
        if (auto1) {
            driveList = aprilTagDetectionPipeline.autoAprilTag(1,12,0);
            drivetrain.drive(driveList.get(0), driveList.get(1), driveList.get(2), DriveSpeedEnum.Auto);

        } else if (auto2) {
            driveList = aprilTagDetectionPipeline.autoAprilTag(2,12,0);
            drivetrain.drive(driveList.get(0), driveList.get(1), driveList.get(2), DriveSpeedEnum.Auto);

        } else {
            DriveSpeedEnum driveSpeed;
            driveList = new ArrayList<Double>();
            if (gamepad1.right_bumper) {
                driveSpeed = DriveSpeedEnum.Fast;
            } else {
                driveSpeed = DriveSpeedEnum.Slow;
            }
            //Testing wheels
            /*
                if (gamepad1.x){
                    drivetrain.rightFrontDrive.setPower(0.25);
                } else {
                    drivetrain.rightFrontDrive.setPower(0);
                }

                if (gamepad1.y){
                    drivetrain.leftFrontDrive.setPower(0.25);
                } else {
                    drivetrain.leftFrontDrive.setPower(0);
                }

                if (gamepad1.b){
                    drivetrain.rightBackDrive.setPower(0.25);
                } else {
                    drivetrain.rightBackDrive.setPower(0);
                }

                if (gamepad1.a){
                    drivetrain.leftBackDrive.setPower(0.25);
                } else {
                    drivetrain.leftBackDrive.setPower(0);
                }
*/
            drivetrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, driveSpeed);
            /*
            outtake.manualSlides(slidePower);
            intakePower = gamepad1.right_stick_y;
            intake.setIntakePower(intakePower);
            outtake.periodic();
            //pre-sets
            if (gamepad1.x) {
                claw.setPower(Constants.Claw.intake);
            } else if (gamepad1.y) {
                claw.setPower(Constants.Claw.outake);
            } else {
                claw.setPower(Constants.Claw.stop);
            }
        if (gamepad1.dpad_right) {
            outtake.presetSlides(Constants.Slides.medSlides);
        }
        if (gamepad1.dpad_up) {
            plane.setPos(Constants.Plane.release);
        } else {
            plane.setPos(Constants.Plane.hold);
        }
             */
        }
        aprilTagDetectionPipeline.telemetryAprilTag();
        telemetry.addData("Drive List", driveList);
    }
    //just shortening code that will be repeated a lot


    @Override
    public void stop() {
//        claw.stopClaw();
//        outtake.stop();
//        intake.stopIntake();
        drivetrain.stopDrive();
        aprilTagDetectionPipeline.AprilTagStop();
    }
}