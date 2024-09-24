package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.arcrobotics.ftclib.purepursuit.*;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Cameras;
import org.firstinspires.ftc.teamcode.subsystems.ClawSensor;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.OpticalOdometry;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.SpecialIntake;
import org.firstinspires.ftc.teamcode.utility.BarcodePosition;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;

@Autonomous(name = "Auto test 2", group = " test")
@Config
public class OtosAuto2 extends OpMode {
    static Pose2d startingPose = new Pose2d(17, -64, new Rotation2d());
    SparkFunOTOS.Pose2D testPose;
    Path myPath;


    Drivetrain drivetrain;
    OpticalOdometry opticalOdometry;

    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        opticalOdometry = new OpticalOdometry(hardwareMap);

        Waypoint p1 = new StartWaypoint(startingPose);
        Waypoint p2 = new GeneralWaypoint(78, -81, 0.75, 0.75, 5);
        Waypoint p3 = new EndWaypoint(new Pose2d(124, -56, new Rotation2d(Math.toRadians(0))), 0.75, 0.75, 5, 0.5, Math.toRadians(20));

        myPath = new Path(p1, p2, p3);
        myPath.init();

        drivetrain.setPoseEstimateOptical(startingPose);

    }

    @Override
    public void init_loop() {
        telemetry.addData("ready 4", true);
    }

    @Override
    public void start() {

        telemetry.addData("done 1", true);
        telemetry.update();

        try {
            sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        int value = 0;

        while (!myPath.isFinished()) {
            if (myPath.timedOut()) {
                try {
                    throw new InterruptedException("Timed out");
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            // return the motor speeds
            double speeds[] = myPath.loop(drivetrain.getPoseEstimateOpticalRegular().getX(), drivetrain.getPoseEstimateOpticalRegular().getY(), drivetrain.getPoseEstimateOpticalRegular().getHeading());

            drivetrain.drive(speeds[0], speeds[1], speeds[2], DriveSpeedEnum.Auto);
            value++;
            telemetry.addData("loop ", value);
            telemetry.update();
        }


        telemetry.addData("done 2", true);
        telemetry.update();
        try {
            sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

    @Override
    public void loop() {
        telemetry.addData("done final", true);

        try {
            sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}