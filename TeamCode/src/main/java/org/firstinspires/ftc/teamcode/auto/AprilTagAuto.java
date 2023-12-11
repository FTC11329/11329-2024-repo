package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utility.RunAfterTime;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetector;
import org.firstinspires.ftc.teamcode.vision.AprilTagIntoPower;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Optional;

@Autonomous(name = "April Tag Auto", group = "Competition")
public class AprilTagAuto extends LinearOpMode {
    WebcamName webcam1;

    Drivetrain drivetrain;
    AprilTagDetector aprilTagDetector;
    AprilTagIntoPower aprilTagIntoPower;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private boolean exactTolerance(int id) {
        Optional<AprilTagDetection> tag = aprilTagDetectionPipeline.getDesiredTag(id);

        if (!tag.isPresent()) return false;

        return aprilTagIntoPower.inToleranceExact(aprilTagDetector.distanceFromAprilTag(tag.get()));
    }

    @Override
    public void runOpMode() {
        //init
        webcam1 = hardwareMap.get(WebcamName.class, Constants.Vision.webcamName);

        drivetrain = new Drivetrain(hardwareMap, telemetry);
        aprilTagDetector = new AprilTagDetector();
        aprilTagIntoPower = new AprilTagIntoPower();
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline();

        aprilTagDetectionPipeline.AprilTagInit(webcam1, telemetry);

        waitForStart();
        //start

        //goes forward for 1 second
        drivetrain.drive(-.5, 0, 0, DriveSpeedEnum.Auto);
        RunAfterTime.runAfterTime(400, () -> {
            drivetrain.drive(0, 0, 0, DriveSpeedEnum.Auto);
        });

        //turns until it sees the tag
        while (!aprilTagDetectionPipeline.getDesiredTag(10).isPresent() && opModeIsActive()) {
            drivetrain.drive(0, 0, 0.3, DriveSpeedEnum.Auto);
        }

        //moves to tag
        //while tag with id 10 is not in tolerance
        while (!aprilTagIntoPower.inTolerance(aprilTagDetector.distanceFromAprilTag(aprilTagDetectionPipeline.getDesiredTag(10).get())) && opModeIsActive()) {
            //creates a variable of an april tag detection
            Optional<AprilTagDetection> optionalAprilTagDetection = aprilTagDetectionPipeline.getDesiredTag(10);

            if (optionalAprilTagDetection.isPresent()) {
                //creates a variable with the pose of the april tag in the form (distance(x and y), yaw, bearing)
                Pose2d distance = aprilTagDetector.distanceFromAprilTag(optionalAprilTagDetection.get());

                //creates a pose of what powers the motors should be set to in the form (forward, strafe, turn)
                Pose2d power = aprilTagIntoPower.toPower(distance);
                //sets that pose to the motors
                drivetrain.setWeightedDrivePower(power);
            }
        }

        //moves exactly to tag
        while (!exactTolerance(10) && opModeIsActive()) {
            //creates a variable of an april tag detection
            Optional<AprilTagDetection> optionalAprilTagDetection = aprilTagDetectionPipeline.getDesiredTag(10);

            if (optionalAprilTagDetection.isPresent()) {
                //creates a variable with the pose of the april tag in the form (distance(x and y), yaw, bearing)
                Pose2d distance = aprilTagDetector.distanceFromAprilTagExact(optionalAprilTagDetection.get());

                //creates a pose of what powers the motors should be set to in the form (forward, strafe, turn)
                Pose2d power = aprilTagIntoPower.toPowerExact(distance);
                //sets that pose to the motors
                drivetrain.setWeightedDrivePower(power);
            }
        }

        //moves to the 2nd tape
    }
}
