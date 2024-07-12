package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp(name = "Drivetrain Only", group = "Helpers")
public class DrivetrainOnly extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        double temp = 0;

        waitForStart();

        while (opModeIsActive()) {
            DriveSpeedEnum speed = gamepad1.right_bumper ? DriveSpeedEnum.Fast : DriveSpeedEnum.Slow;
            drivetrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, speed);
            telemetry.addData("Drivetrain pose", drivetrain.getPoseEstimate().toString());

            temp += (gamepad1.right_trigger - gamepad1.left_trigger) * 0.01;
            telemetry.addData("temp", temp);
            outtake.setWristPos(1);
            outtake.setExtendo(outtake.extendo.mmToRotation(temp));
            telemetry.addData("mm", outtake.extendo.mmToRotation(temp));
            telemetry.addData("Funny:", Class.class.getClasses());

            telemetry.update();
        }
    }


}
