package org.firstinspires.ftc.teamcode.helpers;

import static org.firstinspires.ftc.teamcode.Constants.Drivetrain.leftFrontHardwareMapName;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain.leftRearHardwareMapName;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain.rightFrontHardwareMapName;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain.rightRearHardwareMapName;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "Test Motors", group = "Helpers")
public class TestMotors extends LinearOpMode {
    public static double POWER = 0.2;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftFront = hardwareMap.get(DcMotorEx.class, leftFrontHardwareMapName);
        DcMotor leftRear = hardwareMap.get(DcMotorEx.class, leftRearHardwareMapName);
        DcMotor rightRear = hardwareMap.get(DcMotorEx.class, rightRearHardwareMapName);
        DcMotor rightFront = hardwareMap.get(DcMotorEx.class, rightFrontHardwareMapName);

        waitForStart();

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Moving Left Front Motor");
        telemetry.update();

        leftFront.setPower(POWER);
        sleep(2000);
        leftFront.setPower(0);

        telemetry.addLine("Moving Right Front Motor");
        telemetry.update();

        rightFront.setPower(POWER);
        sleep(2000);
        rightFront.setPower(0);

        telemetry.addLine("Moving Left Rear Motor");
        telemetry.update();

        leftRear.setPower(POWER);
        sleep(2000);
        leftRear.setPower(0);

        telemetry.addLine("Moving Right Rear Motor");
        telemetry.update();

        rightRear.setPower(POWER);
        sleep(2000);
        rightRear.setPower(0);

        telemetry.addLine("Switching to manual drive...");
        telemetry.update();
        sleep(500);

        while (opModeIsActive()) {
            double forward = gamepad1.left_stick_y * POWER;
            double strafe = gamepad1.left_stick_x * POWER;
            double rotate = gamepad1.right_stick_x * POWER;
            leftFront.setPower(forward - strafe + rotate);
            leftRear.setPower(forward + strafe + rotate);

            rightFront.setPower(forward - strafe - rotate);
            rightRear.setPower(forward + strafe - rotate);
        }

    }
}
