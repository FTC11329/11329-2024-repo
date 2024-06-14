package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.utility.SequenceFunction;

public class ConstantCRIPaths {
    @Config
    public static final class PlacePurpleRed {

    }
    @Config
    public static final class PlacePathsRed {
        static SequenceFunction testSequence = (prev) -> {
            prev
            .waitSeconds(1);
        };
    }
}
