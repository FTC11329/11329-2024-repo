package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp(name = "Drivetrain Only", group = "Helpers")
public class DrivetrainOnly extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            DriveSpeedEnum speed = gamepad1.right_bumper ? DriveSpeedEnum.Fast : DriveSpeedEnum.Slow;
            drivetrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, speed);
            telemetry.addData("Drivetrain pose", drivetrain.getPoseEstimate().toString());

            telemetry.update();
        }
    }


}
