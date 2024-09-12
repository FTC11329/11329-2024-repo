package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AprilTagToRoadRunner {
    public static Pose2d tagToRunner(AprilTagDetection tag, boolean isBack) {
        double VxSign = 1;

        double Vx = tag.ftcPose.x;
        double Vy = tag.ftcPose.y;
        double Vh = Math.toRadians(90 - tag.ftcPose.yaw);

        if (isBack) {
            Vx -= Constants.Vision.camOffsetBack.getX();
            Vy -= Constants.Vision.camOffsetBack.getY();
            Vh -= Constants.Vision.camOffsetBack.getHeading();
        } else {
            Vx -= Constants.Vision.camOffsetFront.getX();
            Vy -= Constants.Vision.camOffsetFront.getY();
            Vh -= Constants.Vision.camOffsetFront.getHeading();

        }

        double Rh = Vh;

        double Rx = Math.abs((Math.sqrt(Math.pow(Vx, 2) + Math.pow(Vy, 2))) * Math.sin(Math.toRadians(90) - (Math.toRadians(180) - Vh - Math.atan(Vy/Vx))));

        double Ry = (Math.sqrt(Math.pow(Vx, 2) + Math.pow(Vy, 2))) * Math.cos(Math.toRadians(90) - (Math.toRadians(180) - Vh - Math.atan(Vy/Vx)));

        //this math is stoiped but is has to be here
        if (Vx < 0) {
            VxSign = -1;
        }

        Pose2d runnerPose = new Pose2d(Rx, Ry * VxSign, Rh);

        if (tag.id <= 6 || tag.id > 10) {
            //Backdrop Tags
            runnerPose = new Pose2d(runnerPose.getX() * -1, runnerPose.getY() , runnerPose.getHeading());
        } else {
            runnerPose = new Pose2d(runnerPose.getX(), runnerPose.getY() * -1, runnerPose.getHeading());
        }

        //haha i made my super long if-tree not a super long if-tree yay

        switch (tag.id) {
            case 1:{
                runnerPose = Constants.Vision.tag1Pose.plus(runnerPose);
                break;
            } case 2:{
                runnerPose = Constants.Vision.tag2Pose.plus(runnerPose);
                break;
            } case 3:{
                runnerPose = Constants.Vision.tag3Pose.plus(runnerPose);
                break;
            } case 4:{
                runnerPose = Constants.Vision.tag4Pose.plus(runnerPose);
                break;
            } case 5:{
                runnerPose = Constants.Vision.tag5Pose.plus(runnerPose);
                break;
            } case 6:{
                runnerPose = Constants.Vision.tag6Pose.plus(runnerPose);
                break;
            } case 7:{
                runnerPose = Constants.Vision.tag7Pose.plus(runnerPose);
                break;
            } case 8:{
                runnerPose = Constants.Vision.tag8Pose.plus(runnerPose);
                break;
            } case 9:{
                runnerPose = Constants.Vision.tag9Pose.plus(runnerPose);
                break;
            } case 10:{
                runnerPose = Constants.Vision.tag10Pose.plus(runnerPose);
                break;
                /*
            } case 11:{
                runnerPose = Constants.Vision.tag11Pose.plus(runnerPose);
                break;
            } case 12:{
                runnerPose = Constants.Vision.tag12Pose.plus(runnerPose);
                break;
            } case 13:{
                runnerPose = Constants.Vision.tag13Pose.plus(runnerPose);
                break;
                
                 */
            }
        }
        //to account for line 14
        if (isBack) {
            runnerPose = runnerPose.plus(new Pose2d(0,0,Math.toRadians(90)));
        } else {
            runnerPose = runnerPose.plus(new Pose2d(0,0,Math.toRadians(-90)));
        }
        return runnerPose;
    }
}
