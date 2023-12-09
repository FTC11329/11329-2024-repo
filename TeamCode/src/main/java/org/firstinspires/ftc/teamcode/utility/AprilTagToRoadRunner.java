package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AprilTagToRoadRunner {
    public static Pose2d tagToRunner(AprilTagDetection tag) {
        double Vx = tag.ftcPose.x;
        double Vy = tag.ftcPose.y;
        double Vh = Math.toRadians(90 - tag.ftcPose.yaw);

        double Rh = Vh;

        double Rx = Math.abs((Math.sqrt(Math.pow(Vx, 2) + Math.pow(Vy, 2))) * Math.sin(Math.toRadians(90) - (Math.toRadians(180) - Vh - Math.atan(Vy/Vx))));

        double Ry = (Math.sqrt(Math.pow(Vx, 2) + Math.pow(Vy, 2))) * Math.cos(Math.toRadians(90) - (Math.toRadians(180) - Vh - Math.atan(Vy/Vx)));

        if (tag.id == 10) {
            Rh = Math.PI - Rh;
            Rx = Rx - 16.5;
            Ry = Ry - 23.5;
        }
        return new Pose2d(Rx, Ry, Rh);
    }
}
