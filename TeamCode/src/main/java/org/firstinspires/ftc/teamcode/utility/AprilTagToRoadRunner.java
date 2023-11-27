package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AprilTagToRoadRunner {
    public static Pose2d tagToRunner(AprilTagDetection tag) {
        Pose2d tagPose = new Pose2d(tag.ftcPose.x, tag.ftcPose.y, 90 - tag.ftcPose.yaw);
        //if V is vision so Vh is vision heading then this is the math in my words:
        //Rx = sqrt(Vx^2 + Vy^2) * sin(Vh + arcsin( (Vx)/(sqrt(Vx^2 + Vy^2) )
        double runnerPoseX = Math.sqrt(Math.pow(tagPose.getX(), 2) + Math.pow(tagPose.getY(), 2)) + Math.sin(tagPose.getHeading() + Math.asin(tagPose.getX()/(Math.sqrt(Math.pow(tagPose.getX(), 2) + Math.pow(tagPose.getY(), 2)))));

        //Ry = sqrt(- Rx^2 + Vx^2 + Vy^2)
        double runnerPoseY = Math.sqrt(-Math.pow(runnerPoseX, 2) + Math.pow(tagPose.getX(), 2) + Math.pow(tagPose.getY(), 2));

        //Rh = Vh + arcsin((Vx)/(sqrt(Vx^2 + Vy^2)
        double runnerPoseHeading = tagPose.getHeading() + Math.asin(tagPose.getX()/(Math.sqrt(Math.pow(tagPose.getX(), 2) + Math.pow(tagPose.getY(), 2))));

        return new Pose2d(runnerPoseX, runnerPoseY, runnerPoseHeading);
    }
}
