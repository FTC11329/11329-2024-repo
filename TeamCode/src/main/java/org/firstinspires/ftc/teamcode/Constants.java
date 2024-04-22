package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.checkerframework.checker.compilermsgs.qual.CompilerMessageKey;
import org.checkerframework.checker.units.qual.C;
import org.opencv.core.Scalar;

/**
 * If we want to be able to change any of the constants, they can not be marked with final
 * <p>
 * It is also required to have the sub-class have @Config on it, to mark it to FTCDashboard
 * that it had values that can be changed.
 */
public class Constants {
    @Config
    public static class Vision {
        public static String frontWebcamName = "Webcam 1";
        public static String backWebcamName  = "Webcam 2";
        public static Scalar blueMin = new Scalar(100, 150, 100);
        public static Scalar blueMax = new Scalar(120, 245, 255);

        public static Scalar redPosMin = new Scalar(5, 70, 100);
        public static Scalar redPosMax = new Scalar(30, 255, 255);

        public static Scalar redNegMin = new Scalar(0, 0, 0);
        public static Scalar redNegMax = new Scalar(0, 0, 0);
        public static double percentThreshold = 5.0;

        public static Pose2d camOffsetBack = new Pose2d(3, -7.5, Math.toRadians(0));
        public static Pose2d camOffsetFront = new Pose2d(8.5, -5.25, Math.toRadians(0));


        public static Pose2d tag1Pose = new Pose2d(62,41.5,0);
        public static Pose2d tag2Pose = new Pose2d(62,35.5,0);
        public static Pose2d tag3Pose = new Pose2d(62,29.5,0);

        public static Pose2d tag4Pose = new Pose2d(62,-29.5,0);
        public static Pose2d tag5Pose = new Pose2d(62,-35.5,0);
        public static Pose2d tag6Pose = new Pose2d(62,-41.5,0);

        public static Pose2d tag7Pose = new Pose2d(-70.5,-40, Math.toRadians(180));
        public static Pose2d tag8Pose = new Pose2d(-70.5,-34.5, Math.toRadians(180));

        public static Pose2d tag9Pose = new Pose2d(-70.5,34.5, Math.toRadians(180));
        public static Pose2d tag10Pose= new Pose2d(-70.5,40, Math.toRadians(180));
    }

    @Config
    public static class Intake {
        public static double intake = 0.9;
        public static double outake = -0.5;

        public static double autoVomitSpeed = -0.2;
    }

    @Config
    public static class SpecialIntake {
        public static double min = 0;
        public static double max = 0.37;

        public static double ready = 0.235;
        public static double up = 0.0666666666666666666666;
        public static double down5 = 0.275;
        public static double down4 = 0.295;
        public static double down3 = 0.315;
        public static double down2 = 0.325;
        public static double down1 = 0.365;

        public static double manualPosition = 0.04;
    }

    @Config
    public static class Claw {
        public static double intake = 1;
        public static double outake = -0.4;
        public static double slowOutake = -0.2;
    }

    @Config
    public static class ClawSensor {
        public static double distanceLimit = 0.65;

        public static Vector3D whiteFront  = new Vector3D(1782, 3420, 4120);
        public static Vector3D whiteBack   = new Vector3D(1425, 2950, 3600);
        public static Vector3D yellowFront = new Vector3D(1000, 1600,  564);
        public static Vector3D yellowBack  = new Vector3D( 771, 1265,  390);
        public static Vector3D greenFront  = new Vector3D( 360, 1155,  565);
        public static Vector3D greenBack   = new Vector3D( 217,  845,  372);
        public static Vector3D purpleFront = new Vector3D( 600, 980, 1985);
        public static Vector3D purpleBack  = new Vector3D(650, 1110, 1990);

    }

    @Config
    public static class Plane {
        public static final String servoName = "planeServo";
        public static double hold = 0.1;
        public static double fire = 0.45;
    }

    @Config
    public static class Slides {
        public static int manualSlidePower = 70;
        public static double slowManualSlidePower = 0.4; //this times manual slide power is the actual speed

        public static int upAmount = 10;

        public static int intake = 0;
        public static int superLow = 500; //450
        public static int low = 900;
        public static int med = 1600;
        public static int high = 1850;

        public static int groundPinchMin = 130;
        public static int groundPinchMax = 400;

        public static int slideMax = 1850;
        public static int slideMin = 0;
    }

    @Config
    public static class Arm {
        public static double armMax = 0.65;
        public static double armMin = 0;

        public static double fixPos = 0.524;
        public static double weirdPlacePos = 0.475;
        public static double placePos = 0.370;
        public static double autoArmDrop = 0.06;
        public static double intakePos = 0.015;

        public static double manualPosition = 0.03;
    }

    @Config
    public static class Climber {
        public static double manualClimberPower = 50;


        public static int down = 0;
        public static int climberFire = -1052;
        public static int climbButSlightlyDown = -1400;
        public static int climb = -1700;
    }

    @Config
    public static class DistanceSensors {
        public static int tolerance = 60;
    }
    @Config
    public static class AutoServo {
        public static double downLeft = 0.92;
        public static double downRight = 0.91;

        public static double upLeft = 0;
        public static double upRight = 0;
    }

    @Config
    public static class Drivetrain {
        public static final String leftEncoderName = "intakeMotor";
        public static final String rightEncoderName = "backRight";
        public static final String frontEncoderName = "frontRight";

        public static final String leftFrontHardwareMapName = "frontLeft";
        public static final String leftRearHardwareMapName = "backLeft";
        public static final String rightFrontHardwareMapName = "frontRight";
        public static final String rightRearHardwareMapName = "backRight";

        public static double superFast = 1;
        public static double fast = 1;
        public static double slow = 0.5;
        public static double fixSpeedStrafe = 0.75;
        public static double fixSpeedForward = 0.15;
    }

    @Config
    public static class Roadrunner {
        public static final double PARALLEL_ENCODER_TICKS = 8192;
        public static final double LATERAL_ENCODER_TICKS = 8192;
        public static final double MAX_RPM = 312;

        /*
         * These are physical constants that can be determined from your robot (including the track
         * width; it will be tune empirically later although a rough estimate is important). Users are
         * free to chose whichever linear distance unit they would like so long as it is consistently
         * used. The default values were selected with inches in mind. Road runner uses radians for
         * angular distances although most angular parameters are wrapped in Math.toRadians() for
         * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
         */
        public static double WHEEL_RADIUS = 1.8898; // in
        public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
        public static double TRACK_WIDTH = 9.8; // in
        /*
         * These are the feedforward parameters used to model the drive motor behavior. If you are using
         * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
         * motor encoders or have elected not to use them for velocity control, these values should be
         * empirically tuned.
         */
        public static double kV = 0.0143; //0.0165
        public static double kA = 0.00248; //0.0015
        public static double kStatic = 0;

        /*
         * These values are used to generate the trajectories for your robot. To ensure proper operation,
         * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
         * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
         * small and gradually increase them later after everything is working. All distance units are
         * inches.
         */
        public static double MAX_VEL = 45;
        public static double MAX_ACCEL = 45;
        public static double MAX_ANG_VEL = Math.toRadians(120);
        public static double MAX_ANG_ACCEL = Math.toRadians(120);
    }
}
