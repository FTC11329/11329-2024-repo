package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.core.Mat;

public class AprilTagToRoadRunner {
    public static Pose2d tagToRunner(AprilTagDetection tag) {
        Pose2d tagPose = new Pose2d(tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.bearing);
        double Vx = tagPose.getX();
        double Vy = tagPose.getY();
        double Vh = tagPose.getHeading();

        //if V is vision so Vh is vision heading then this is the math in my words:
        //Rx = sqrt(Vx^2 + Vy^2) * sin(Vh + arcsin( (Vx)/(sqrt(Vx^2 + Vy^2) )
//        double runnerPoseX = Math.sqrt(Math.pow(tagPose.getX(), 2) + Math.pow(tagPose.getY(), 2)) + Math.sin(tagPose.getHeading() + Math.asin(tagPose.getX()/(Math.sqrt(Math.pow(tagPose.getX(), 2) + Math.pow(tagPose.getY(), 2)))));

        //Ry = sqrt(- Rx^2 + abs(Vx^2 + Vy^2))
//        double runnerPoseY = Math.sqrt(-Math.pow(runnerPoseX, 2) + Math.abs(Math.pow(tagPose.getX(), 2) + Math.pow(tagPose.getY(), 2)));

        //Rh = Vh + arcsin((Vx)/(sqrt(Vx^2 + Vy^2)
//        double runnerPoseHeading = tagPose.getHeading() + Math.asin(tagPose.getX()/(Math.sqrt(Math.pow(tagPose.getX(), 2) + Math.pow(tagPose.getY(), 2))));

        double Rx = (Vy - (Vx * Math.tan(Vh)) * Math.sin(Vh))/(Math.tan(Vh));

        double Ry = (Vy - Vx * Math.tan(Vh) * Math.sin(Vh) + Vx * Math.tan(Vh));

        double Rh = 0;

        return new Pose2d(Vx, Vy, tag.ftcPose.yaw);
    }
}
