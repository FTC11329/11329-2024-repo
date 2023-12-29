package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AprilTagToRoadRunner {
    public static Pose2d tagToRunner(AprilTagDetection tag) {
        double Vx = tag.ftcPose.x;
        double Vy = tag.ftcPose.y;
        double Vh = Math.toRadians(90 - tag.ftcPose.yaw);

//        Vx -= Constants.Vision.camOffset.getX();
//        Vy -= Constants.Vision.camOffset.getY();
//        Vh -= Constants.Vision.camOffset.getHeading();

        double Rh = Vh;

        double Rx = Math.abs((Math.sqrt(Math.pow(Vx, 2) + Math.pow(Vy, 2))) * Math.sin(Math.toRadians(90) - (Math.toRadians(180) - Vh - Math.atan(Vy/Vx))));

        double Ry = (Math.sqrt(Math.pow(Vx, 2) + Math.pow(Vy, 2))) * Math.cos(Math.toRadians(90) - (Math.toRadians(180) - Vh - Math.atan(Vy/Vx)));

        Pose2d runnerPose = new Pose2d(Rx, Ry, Rh);

        //haha i made my super long if-tree not a super long if-tree yay
        switch (tag.id) {
            case 1:{
                runnerPose = runnerPose.minus(Constants.Vision.tag1Pose);
                break;
            } case 2:{
                runnerPose = runnerPose.minus(Constants.Vision.tag2Pose);
                break;
            } case 3:{
                runnerPose = runnerPose.minus(Constants.Vision.tag3Pose);
                break;
            } case 4:{
                runnerPose = runnerPose.minus(Constants.Vision.tag4Pose);
                break;
            } case 5:{
                runnerPose = runnerPose.minus(Constants.Vision.tag5Pose);
                break;
            } case 6:{
                runnerPose = runnerPose.minus(Constants.Vision.tag6Pose);
                break;
            } case 7:{
                runnerPose = runnerPose.minus(Constants.Vision.tag7Pose);
                break;
            } case 8:{
                runnerPose = runnerPose.minus(Constants.Vision.tag8Pose);
                break;
            } case 9:{
                runnerPose = runnerPose.minus(Constants.Vision.tag9Pose);
                break;
            } case 10:{
                runnerPose = runnerPose.minus(Constants.Vision.tag10Pose);
                break;
            }
        }
        return runnerPose;
    }
}
