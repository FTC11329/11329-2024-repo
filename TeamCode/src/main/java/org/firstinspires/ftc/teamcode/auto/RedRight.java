package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Cameras;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.utility.BarcodePosition;

@Autonomous(name = "Red Right")
public class RedRight extends LinearOpMode {
    Pose2d startingPosition = new Pose2d(8, -50);

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        Outtake outtake = new Outtake(hardwareMap);
        Cameras cameras = new Cameras(hardwareMap);


        waitForStart();

        BarcodePosition barcodePosition = cameras.getBarcodePosition().orElse(BarcodePosition.One);
        drivetrain.setPoseEstimate(startingPosition);
        TrajectorySequence toBackboard = drivetrain
                .trajectorySequenceBuilder(startingPosition)
                .build();


    }
}
