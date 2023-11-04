package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveUtilityMethods.getMotorVelocityF;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveUtilityMethods.rpmToVelocity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Constants {
    @Config
    public static class Claw {
        public static final double intake = 0.5;
        public static final double outake =-0.5;
        public static final double stop = 0;
    }

    @Config
    public static class Plane {
        public static final double hold = 0;
        public static final double release = 0.3;
    }

    @Config
    public static class Slides {
        public static final int medSlides = 1000;
    }

    @Config
    public static class Drivetrain {
        public static String leftFrontHardwareMapName = "frontLeft";
        public static String leftRearHardwareMapName = "backLeft";
        public static String rightFrontHardwareMapName =  "frontRight";
        public static String rightRearHardwareMapName = "backRight";
    }

    @Config
    public static class Roadrunner {
        public static final double TICKS_PER_REV = 1;
        public static final double MAX_RPM = 1;

        /*
         * These are physical constants that can be determined from your robot (including the track
         * width; it will be tune empirically later although a rough estimate is important). Users are
         * free to chose whichever linear distance unit they would like so long as it is consistently
         * used. The default values were selected with inches in mind. Road runner uses radians for
         * angular distances although most angular parameters are wrapped in Math.toRadians() for
         * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
         */
        public static double WHEEL_RADIUS = 2; // in
        public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
        public static double TRACK_WIDTH = 1; // in

        /*
         * These are the feedforward parameters used to model the drive motor behavior. If you are using
         * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
         * motor encoders or have elected not to use them for velocity control, these values should be
         * empirically tuned.
         */
        public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
        public static double kA = 0;
        public static double kStatic = 0;

        /*
         * These values are used to generate the trajectories for you robot. To ensure proper operation,
         * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
         * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
         * small and gradually increase them later after everything is working. All distance units are
         * inches.
         */
        public static double MAX_VEL = 30;
        public static double MAX_ACCEL = 30;
        public static double MAX_ANG_VEL = Math.toRadians(60);
        public static double MAX_ANG_ACCEL = Math.toRadians(60);
    }

}
