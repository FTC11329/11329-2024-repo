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
import java.util.List;


@TeleOp(name="Allen Test Drive", group="Allen op mode")
public class Teleop extends OpMode
{
    WebcamName webcam1;

    private ElapsedTime runtime = new ElapsedTime();

    Drivetrain drivetrain;
    Intake intake;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private boolean auto = false;

    double intakePower;
    @Override
    public void init() {
        intakePower = 0;
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline();
        telemetry.addData("Status", "Initialized");
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        aprilTagDetectionPipeline.AprilTagInit(webcam1, telemetry);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        auto = gamepad1.a;
        List<Double> driveList = aprilTagDetectionPipeline.moveToAprilTag();
        if (!auto) {
            if (gamepad1.right_bumper) {
                drivetrain.driveSpeed = DriveSpeedEnum.Fast;
            } else {
                drivetrain.driveSpeed = DriveSpeedEnum.Slow;
            }
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
            */
            drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            drivetrain.driveSpeed = DriveSpeedEnum.Auto;
            drivetrain.drive(driveList.get(0), driveList.get(1), driveList.get(2));
        }

        intakePower = gamepad1.left_trigger - gamepad1.right_trigger;
        intake.setIntakePower(intakePower);
        aprilTagDetectionPipeline.telemetryAprilTag();
        telemetry.addData("forward", driveList.get(0));
        telemetry.addData("strafe" , driveList.get(1));
        telemetry.addData("turn"   , driveList.get(2));

    }

    @Override
    public void stop() {
        drivetrain.stopDrive();
        aprilTagDetectionPipeline.AprilTagStop();
    }
}