package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Cameras;
import org.firstinspires.ftc.teamcode.subsystems.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utility.RunAfterTime;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetector;
import org.firstinspires.ftc.teamcode.vision.AprilTagIntoPower;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Optional;

@Autonomous(name = "April Tag Auto", group = "Testing")
public class AprilTagAuto extends LinearOpMode {
    Drivetrain drivetrain;
    Cameras cameras;

    private boolean exactTolerance(int id) {
        Optional<Pose2d> tag = cameras.getRunnerPoseEstimate(id, true);
        if (!tag.isPresent()) return false;

        Pose2d tagPose = tag.get();

        return AprilTagIntoPower.inToleranceExact(tagPose);
    }

    @Override
    public void runOpMode() {
        //init
        cameras = new Cameras(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, telemetry);

        waitForStart();
        //start

        //goes forward for 1 second
        drivetrain.drive(-.5, 0, 0, DriveSpeedEnum.Auto);
        RunAfterTime.runAfterTime(400, () -> {
            drivetrain.drive(0, 0, 0, DriveSpeedEnum.Auto);
        });

        //turns until it sees the tag
        while (!AprilTagDetectionPipeline.getDesiredTag(cameras.getAprilTagRecognitions(), 10).isPresent() && opModeIsActive()) {
            drivetrain.drive(0, 0, 0.3, DriveSpeedEnum.Auto);
        }

        //moves to tag
        //while tag with id 10 is not in tolerance
        while (!AprilTagIntoPower.inTolerance(AprilTagDetector.distanceFromAprilTag(AprilTagDetectionPipeline.getDesiredTag(cameras.getAprilTagRecognitions(), 10).get())) && opModeIsActive()) {
            //creates a variable of an april tag detection
            Optional<AprilTagDetection> optionalAprilTagDetection = AprilTagDetectionPipeline.getDesiredTag(cameras.getAprilTagRecognitions(), 10);

            if (optionalAprilTagDetection.isPresent()) {
                //creates a variable with the pose of the april tag in the form (distance(x and y), yaw, bearing)
                Pose2d distance = AprilTagDetector.distanceFromAprilTag(optionalAprilTagDetection.get());

                //creates a pose of what powers the motors should be set to in the form (forward, strafe, turn)
                Pose2d power = AprilTagIntoPower.toPower(distance);
                //sets that pose to the motors
                drivetrain.setWeightedDrivePower(power);
            }
        }

        //moves exactly to tag
        while (!exactTolerance(10) && opModeIsActive()) {
            //creates a variable of an april tag detection
            Optional<AprilTagDetection> optionalAprilTagDetection = AprilTagDetectionPipeline.getDesiredTag(cameras.getAprilTagRecognitions(), 10);

            if (optionalAprilTagDetection.isPresent()) {
                //creates a variable with the pose of the april tag in the form (distance(x and y), yaw, bearing)
                Pose2d distance = AprilTagDetector.distanceFromAprilTagExact(optionalAprilTagDetection.get());

                //creates a pose of what powers the motors should be set to in the form (forward, strafe, turn)
                Pose2d power = AprilTagIntoPower.toPowerExact(distance);
                //sets that pose to the motors
                drivetrain.setWeightedDrivePower(power);
            }
        }

        //moves to the 2nd tape
    }
}
