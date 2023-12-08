package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveUtilityMethods.rpmToVelocity;

import com.acmerobotics.dashboard.config.Config;

/**
 * If we want to be able to change any of the constants, they can not be marked with final
 * <p>
 * It is also required to have the sub-class have @Config on it, to mark it to FTCDashboard
 * that it had values that can be changed.
 */
public class Constants {
    @Config
    public static class Intake {
        public static double intake = 0.75; //TODO: Set value
        public static double outake = -1;
    }
    @Config
    public static class Claw {
        public static double intake = 1;
        public static double outake = -1;
    }

    @Config
    public static class Plane {
        public static double hold = 0; //TODO: Set value
        public static double release = 0.3; //TODO: Set value
    }

    @Config
    public static class Slides {
        public static int highSlides = 0;//TODO: Set value

        public static int medSlides = 0; //TODO: Set value

        public static int lowSlides = 0; //TODO: Set value


        public static double groundPinchMax = 100000; //TODO: Set value
        public static double groundPinchMin = 100000; //TODO: Set value
    }
    @Config
    public static class Arm{
        public static double groundPinchMax = 100000; //TODO: Set value
        public static double groundPinchMin = 100000; //TODO: Set value

        public static double safePos = 0; //TODO: Set value
    }

    @Config
    public static class Drivetrain {
        public static final String leftFrontHardwareMapName  = "frontLeft";
        public static final String leftRearHardwareMapName   = "backLeft";
        public static final String rightFrontHardwareMapName = "frontRight";
        public static final String rightRearHardwareMapName  = "backRight";
    }

    @Config
    public static class Roadrunner {
        public static final double TICKS_PER_REV = 8192;
        public static final double MAX_RPM = 312;

        /*
         * These are physical constants that can be determined from your robot (including the track
         * width; it will be tune empirically later although a rough estimate is important). Users are
         * free to chose whichever linear distance unit they would like so long as it is consistently
         * used. The default values were selected with inches in mind. Road runner uses radians for
         * angular distances although most angular parameters are wrapped in Math.toRadians() for
         * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
         */
        // TODO: Rework these values for Avalanche
        public static double WHEEL_RADIUS = 1.8898; // in
        public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
        public static double TRACK_WIDTH = 11.125; // in
        /*
         * These are the feedforward parameters used to model the drive motor behavior. If you are using
         * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
         * motor encoders or have elected not to use them for velocity control, these values should be
         * empirically tuned.
         */
        public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
        public static double kA = 0.004;
        public static double kStatic = 0;

        /*
         * These values are used to generate the trajectories for your robot. To ensure proper operation,
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
