package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Drivetrain
{
    DcMotor leftFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    DcMotor rightFrontDrive;
    Telemetry telemetry;
    private double speed = 0.3;
    public DriveSpeedEnum driveSpeed = DriveSpeedEnum.Slow;

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "backLeft" );
        rightBackDrive  = hardwareMap.get(DcMotor.class, "backRight");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");

        leftFrontDrive .setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive  .setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive .setDirection(DcMotor.Direction.FORWARD);
    }
    public void drive(double forward, double strafe, double turn) {
        if (driveSpeed == DriveSpeedEnum.Fast){
            speed = 0.85;
        } else if (driveSpeed == DriveSpeedEnum.Slow) {
            speed = 0.5;
        } else if (driveSpeed == DriveSpeedEnum.Auto) {
            speed = 0.8;
        }

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);
        double frontLeftPower  = (forward + strafe + turn) / denominator;
        double backLeftPower   = (forward - strafe + turn) / denominator;
        double frontRightPower = (forward - strafe - turn) / denominator;
        double backRightPower  = (forward + strafe - turn) / denominator;

        leftFrontDrive .setPower(frontLeftPower * speed);
        leftBackDrive  .setPower(backLeftPower  * speed);
        rightBackDrive .setPower(frontRightPower* speed);
        rightFrontDrive.setPower(backRightPower * speed);
        telemetry.addData("LF", frontLeftPower * speed);
        telemetry.addData("RF", frontRightPower* speed);
        telemetry.addData("LB", backLeftPower  * speed);
        telemetry.addData("RB", backRightPower * speed);

    }
    public void stopDrive() {
        leftFrontDrive .setPower(0);
        leftBackDrive  .setPower(0);
        rightBackDrive .setPower(0);
        rightFrontDrive.setPower(0);
        telemetry.addLine("stopped");
    }

}