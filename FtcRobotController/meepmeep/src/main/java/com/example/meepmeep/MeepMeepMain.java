package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepMain {
    static Pose2d redRightStartingPosition = new Pose2d(37, 60, Math.toRadians(90));
    static Pose2d redLeftStartingPosition = new Pose2d(-37, -60, Math.toRadians(90));
    static Pose2d startingPosition = redLeftStartingPosition;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(12, 16).setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11).followTrajectorySequence(MeepMeepMain::redRight);

        myBot.setPose(startingPosition);

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK).setDarkMode(true).setBackgroundAlpha(0.95f).addEntity(myBot).start();
    }

    public static TrajectorySequence redRight(DriveShim drive) {
        return drive.trajectorySequenceBuilder(startingPosition).build();
    }
}