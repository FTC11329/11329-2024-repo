package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.AprilTagIntoPower.DISTANCE;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

public class AprilTagDetector {
    List<Double> driveList = new ArrayList<>();

    public Pose2d distanceFromAprilTag(AprilTagDetection tag) {
        driveList = new ArrayList<>();
        // Step through the list of detected tags and look for a matching tag
        // Determine range, yaw, and heading (tag image rotation) error so we can use them to control the robot automatically
        double rangeError = tag.ftcPose.range;
        double yawError = tag.ftcPose.yaw;
        double headingError = tag.ftcPose.bearing;

        return new Pose2d(-rangeError, yawError, -headingError);
    }

    public Pose2d distanceFromAprilTagExact(AprilTagDetection tag) {
        driveList = new ArrayList<>();

        double yError = tag.ftcPose.y;
        double xError = tag.ftcPose.x;
        double yawError = tag.ftcPose.yaw;

        return new Pose2d(-yError, xError, -yawError);
    }
}

