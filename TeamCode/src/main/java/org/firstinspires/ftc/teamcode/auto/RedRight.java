package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

public class RedRight extends LinearOpMode {
    Pose2d startingPosition = new Pose2d(8, -50);

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        Outtake outtake = new Outtake(hardwareMap);

        drivetrain.setPoseEstimate(startingPosition);
//        TrajectorySequence toBackboard = drivetrain.trajectorySequenceBuilder(startingPosition);

        waitForStart();


    }
}
