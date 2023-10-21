package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotAutoDriveToLine_Linear;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Drivetrain;

import java.util.ArrayList;
import java.util.List;
import java.lang.Math;


@TeleOp(name="Allen Test Drive", group="Allen op mode")
public class Teleop extends OpMode
{
    WebcamName webcam1;
    Drivetrain drivetrain;
    Intake intake;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private boolean auto1 = false;
    private boolean auto2 = false;

    double intakePower;
    @Override
    public void init() {
        intakePower = 0;
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline();
        telemetry.addData("Status", "Initialized");
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        aprilTagDetectionPipeline.AprilTagInit(webcam1, telemetry);
    }

    @Override
    public void start() {}

    @Override
    public void loop() {
        auto1 = gamepad1.a;
        auto2 = gamepad1.b;
        List<Double> driveList;
        if (auto1) {
            drivetrain.driveSpeed = DriveSpeedEnum.Auto;
            driveList = aprilTagDetectionPipeline.moveToBackdrop();
            drivetrain.drive(driveList.get(0), driveList.get(1), driveList.get(2), true);
        } else if (auto2) {
            drivetrain.driveSpeed = DriveSpeedEnum.Auto;
            driveList = aprilTagDetectionPipeline.moveToTruss();
            drivetrain.drive(driveList.get(0), driveList.get(1), driveList.get(2), true);
        } else {
            driveList = new ArrayList<Double>();
            if (gamepad1.right_bumper) {
                drivetrain.driveSpeed = DriveSpeedEnum.Fast;
            } else {
                drivetrain.driveSpeed = DriveSpeedEnum.Slow;
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
            drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);
        }
        intakePower = gamepad1.left_trigger - gamepad1.right_trigger;
        intake.setIntakePower(intakePower);
        aprilTagDetectionPipeline.telemetryAprilTag();
        telemetry.addData("Drive List", driveList);
    }

    @Override
    public void stop() {
        drivetrain.stopDrive();
        aprilTagDetectionPipeline.AprilTagStop();
    }
}