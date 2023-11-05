package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepMain {
    static Pose2d redLeftStartingPosition = new Pose2d(37, -60, Math.toRadians(90));
    static Pose2d startingPosition = redLeftStartingPosition;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11).followTrajectorySequence(MeepMeepMain::redRight);

        myBot.setPose(startingPosition);


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK).setDarkMode(true).setBackgroundAlpha(0.95f).addEntity(myBot).start();
    }

    public static TrajectorySequence redRight(DriveShim drive) {
        return drive.trajectorySequenceBuilder(startingPosition).splineTo(new Vector2d(45, -35), 0).back(100).waitSeconds(0.5).forward(100).build();
    }
}