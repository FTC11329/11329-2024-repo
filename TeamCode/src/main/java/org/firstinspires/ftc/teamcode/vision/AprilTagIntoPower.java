package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.Range;

public class AprilTagIntoPower {

    final public static double DISTANCE = 15;
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  = 0.03;     //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.035;  //  Strafe Speed Control "Gain". eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   = 0.01;    //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED  = 0.5;    //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;  //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN   = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    double drive;         // Desired forward power/speed (-1 to +1)
    double strafe;       // Desired strafe power/speed (-1 to +1)
    double turn;        // Desired turning power/speed (-1 to +1)

    public Pose2d toPower(Pose2d distanceFromTag) {
        drive = Range.clip((distanceFromTag.getX() + DISTANCE) * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        strafe = Range.clip(distanceFromTag.getY() * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        turn = Range.clip(distanceFromTag.getHeading() * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        return new Pose2d(drive, strafe, turn);
    }

    final double SPEED_GAIN_EXACT  = 0.15;
    final double STRAFE_GAIN_EXACT = 0.15;
    final double TURN_GAIN_EXACT   = 0.03;

    public Pose2d toPowerExact(Pose2d distanceFromTag) {
        drive = Range.clip((distanceFromTag.getX() + DISTANCE) * SPEED_GAIN_EXACT, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        strafe = Range.clip(distanceFromTag.getY() * STRAFE_GAIN_EXACT, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        turn = Range.clip(distanceFromTag.getHeading() * TURN_GAIN_EXACT, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        return new Pose2d(drive, strafe, turn);
    }

    final double tolerance = 0.15;
    final double toleranceExact = 0.05;

    public boolean inTolerance(Pose2d distanceFromTag) {
        Pose2d power = toPower(distanceFromTag);
        if (Math.abs(power.getX()) + Math.abs(power.getY()) + Math.abs(power.getHeading()) < tolerance) {
            return true;
        } else {
            return false;
        }
    }
    public boolean inToleranceExact(Pose2d distanceFromTag) {
        Pose2d power = toPower(distanceFromTag);
        if (Math.abs(power.getX()) + Math.abs(power.getY()) + Math.abs(power.getHeading()) < toleranceExact) {
            return true;
        } else {
            return false;
        }
    }

}
