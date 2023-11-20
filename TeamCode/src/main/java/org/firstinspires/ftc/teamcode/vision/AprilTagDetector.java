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
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically
        double rangeError = tag.ftcPose.range;
        double yawError = tag.ftcPose.yaw;
        double headingError = tag.ftcPose.bearing;

        return new Pose2d(-rangeError, yawError, -headingError);
    }

    final double inchTolerance = 1;
    final double degreesTolerance = 3;

    public boolean inTolerance(AprilTagDetection tag) {
        if (Math.abs(tag.ftcPose.range + DISTANCE) < inchTolerance && Math.abs(tag.ftcPose.bearing) < degreesTolerance) {
            return true;
        } else {
            return false;
        }
    }
}

