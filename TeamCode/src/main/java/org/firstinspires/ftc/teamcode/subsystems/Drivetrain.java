package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Drivetrain.leftFrontHardwareMapName;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain.leftRearHardwareMapName;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain.rightFrontHardwareMapName;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain.rightRearHardwareMapName;
import static org.firstinspires.ftc.teamcode.Constants.Roadrunner.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.Constants.Roadrunner.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.Constants.Roadrunner.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.Constants.Roadrunner.MAX_VEL;
import static org.firstinspires.ftc.teamcode.Constants.Roadrunner.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.Constants.Roadrunner.kA;
import static org.firstinspires.ftc.teamcode.Constants.Roadrunner.kStatic;
import static org.firstinspires.ftc.teamcode.Constants.Roadrunner.kV;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.arcrobotics.ftclib.purepursuit.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.utility.HardwareUtils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class Drivetrain extends MecanumDrive {
    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(9, 5.5, 2); //D3
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(9, 0, 0);
    public static double LATERAL_MULTIPLIER = 1;
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;
    private final TrajectorySequenceRunner trajectorySequenceRunner;
    private final TrajectoryFollower follower;
    public final DcMotorEx leftFront;
    public final DcMotorEx leftRear;
    public final DcMotorEx rightRear;
    public final DcMotorEx rightFront;
    private final List<DcMotorEx> motors;

    private final VoltageSensor batteryVoltageSensor;

    private final List<Integer> lastEncoderPositions = new ArrayList<>();
    private final List<Integer> lastEncoderVelocities = new ArrayList<>();

    private Telemetry telemetry;
    private HardwareMap hardwareMapLocal;
    private OpticalOdometry opticalOdometry;

    public Drivetrain(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        hardwareMapLocal = hardwareMap;
        opticalOdometry = new OpticalOdometry(hardwareMap);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        HardwareUtils.configureHardwareMap(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontHardwareMapName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearHardwareMapName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearHardwareMapName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontHardwareMapName);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        List<Integer> lastTrackingEncoderPositions = new ArrayList<>();
        List<Integer> lastTrackingEncoderVelocities = new ArrayList<>();

        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncoderPositions, lastTrackingEncoderVelocities));

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncoderPositions, lastEncoderVelocities, lastTrackingEncoderPositions, lastTrackingEncoderVelocities
        );




    }


    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public void drive(double forward, double strafe, double turn, DriveSpeedEnum driveSpeed) {
        double speed = 0;
        if (driveSpeed == DriveSpeedEnum.Fast) {
            speed = Constants.Drivetrain.fast;
        } else if (driveSpeed == DriveSpeedEnum.Slow) {
            speed = Constants.Drivetrain.slow;
        } else if (driveSpeed == DriveSpeedEnum.Auto) {
            speed = 1;
        } else if (driveSpeed == DriveSpeedEnum.SuperFast) {
            speed = 0.3;
        }

        setWeightedDrivePower(new Pose2d(forward * speed, strafe * speed, turn * speed));
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(opticalOdometry.getPoseEstimateOpticalRegular())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(opticalOdometry.getPoseEstimateOpticalRegular(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }
    //  This is an artifact that we don't use due to 3 wheel odometry
    @Override
    protected double getRawExternalHeading() {
        return 0;
    }

    // TODO: Refactor this out
    public void stopDrive() {
        drive(0, 0, 0, DriveSpeedEnum.Slow);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return null;
    }

    public com.arcrobotics.ftclib.drivebase.MecanumDrive ArcMeccanumDrive() {
        Motor fL = new Motor(hardwareMapLocal, leftFrontHardwareMapName);
        Motor fR = new Motor(hardwareMapLocal, rightFrontHardwareMapName);
        Motor bL = new Motor(hardwareMapLocal, leftRearHardwareMapName);
        Motor bR = new Motor(hardwareMapLocal, rightRearHardwareMapName);
        return new com.arcrobotics.ftclib.drivebase.MecanumDrive(fL, fR, bL, bR);
    }

    //Pass through functions

    public Pose2d getPoseEstimateOpticalRegular() {
        return opticalOdometry.getPoseEstimateOpticalRegular();
    }
    public SparkFunOTOS.Pose2D getPoseEstimateOptical(){
        return opticalOdometry.getPoseEstimateOptical();
    }
    public void setPoseEstimateOptical(com.arcrobotics.ftclib.geometry.Pose2d newPose){
        opticalOdometry.setPoseEstimateOptical(newPose);
    }
    public void setPoseEstimateOptical(Pose2d newPose) {
        setPoseEstimateOptical(new com.arcrobotics.ftclib.geometry.Pose2d(newPose.getX(), newPose.getY(), new Rotation2d(newPose.getHeading())));
    }

}
