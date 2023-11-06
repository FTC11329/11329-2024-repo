package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Autonomous(name = "Red Right", group = "Competition")
@Config
public class RedRight extends LinearOpMode {
    static Pose2d startingPose = new Pose2d(37, -60, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);

        Trajectory goToBackboard = drivetrain.trajectoryBuilder(startingPose).splineTo(new Vector2d(45, -35), 0).build();
        Trajectory goToStack = drivetrain.trajectoryBuilder(goToBackboard.end()).back(100).build();
        Trajectory goBackToBackboard = drivetrain.trajectoryBuilder(goToStack.end()).forward(100).build();

        waitForStart();

        drivetrain.followTrajectory(goToBackboard);

        sleep(500);

        drivetrain.followTrajectory(goToStack);

        sleep(500);

        drivetrain.followTrajectory(goBackToBackboard);
    }
}
