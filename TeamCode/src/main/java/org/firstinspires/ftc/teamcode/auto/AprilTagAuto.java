package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

    AprilTagDetector aprilTagDetector;
    AprilTagIntoPower aprilTagIntoPower;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);

    @Override
    public void runOpMode() {
        //init

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilTagDetector = new AprilTagDetector();
        aprilTagIntoPower = new AprilTagIntoPower();
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline();

        aprilTagDetectionPipeline.AprilTagInit(webcam1, telemetry);

        waitForStart();
        //start

        //goes forward for 1 second
        drivetrain.drive(.5,0,0, DriveSpeedEnum.Auto);
        RunAfterTime.runAfterTime(1000,() -> {
            drivetrain.drive(0,0,0, DriveSpeedEnum.Auto);
        });

        //turns until it sees the tag
        while (!aprilTagDetectionPipeline.getDesiredTag(10).isPresent()){
            drivetrain.drive(0,0,0.3, DriveSpeedEnum.Auto);
        }

        //moves to tag
        //while tag with id 10 is in tolerance
        while (!aprilTagDetector.inTolerance(aprilTagDetectionPipeline.getDesiredTag(10).get())) {
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
    }
}
