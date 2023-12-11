package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AprilTagDetector {
    public static Pose2d distanceFromAprilTag(AprilTagDetection tag) {
        // Step through the list of detected tags and look for a matching tag
        // Determine range, yaw, and heading (tag image rotation) error so we can use them to control the robot automatically
        double rangeError = tag.ftcPose.range;
        double yawError = tag.ftcPose.yaw;
        double headingError = tag.ftcPose.bearing;

        return new Pose2d(-rangeError, yawError, -headingError);
    }

    public static Pose2d distanceFromAprilTagExact(AprilTagDetection tag) {
        double yError = tag.ftcPose.y;
        double xError = tag.ftcPose.x;
        double yawError = tag.ftcPose.yaw;

        return new Pose2d(-yError, xError, -yawError);
    }
}

