package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AprilTagToRoadRunner {
    public static Pose2d tagToRunner(AprilTagDetection tag) {
        double Vx = tag.ftcPose.x;
        double Vy = tag.ftcPose.y;
        double Vh = Math.toRadians(90 - tag.ftcPose.yaw);

        //if V is vision so Vh is vision heading then this is the math in my words:
        //Rx = sqrt(Vx^2 + Vy^2) * sin(Vh + arcsin( (Vx)/(sqrt(Vx^2 + Vy^2) )
//        double Rx = Math.sqrt(Math.pow(Vx, 2) + Math.pow(Vy, 2)) + Math.sin(hV + Math.asin(Vx/(Math.sqrt(Math.pow(Vx, 2) + Math.pow(Vy, 2)))));

        //Ry = sqrt(- Rx^2 + abs(Vx^2 + Vy^2))
//        double runnerPoseY = Math.sqrt(-Math.pow(runnerPoseX, 2) + Math.abs(Math.pow(tagPose.getX(), 2) + Math.pow(tagPose.getY(), 2)));

        //Rh = Vh + arcsin((Vx)/(sqrt(Vx^2 + Vy^2)
//        double runnerPoseHeading = tagPose.getHeading() + Math.asin(tagPose.getX()/(Math.sqrt(Math.pow(tagPose.getX(), 2) + Math.pow(tagPose.getY(), 2))));

        double Ry = (Vy - Vx * Math.tan(Vh) * Math.sin(Vh) + Vx * Math.tan(Vh));

        double Rx = ((Vy - (Vx * Math.tan(Vh))) * (Math.cos(Vh)));

        double Rh = Vh;

        return new Pose2d(Rx, Ry, Rh);
    }
}
