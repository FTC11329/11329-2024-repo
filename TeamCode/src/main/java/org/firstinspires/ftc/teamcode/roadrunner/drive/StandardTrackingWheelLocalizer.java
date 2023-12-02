package org.firstinspires.ftc.teamcode.roadrunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.6889764; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 10.75; // in; distance between the left and right wheels
    public static double SIDE_FORWARD_OFFSET = 0.125; // in; offset of the lateral wheel
    public static double FORWARD_OFFSET = -2.5;
    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private final Encoder frontEncoder;

    private final List<Integer> lastEncoderPositions;
    private final List<Integer> lastEncoderVelocities;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, List<Integer> lastTrackingEncoderPositions, List<Integer> lastTrackingEncoderVelocities) {
        super(Arrays.asList(
                new Pose2d(SIDE_FORWARD_OFFSET, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(SIDE_FORWARD_OFFSET, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        lastEncoderPositions = lastTrackingEncoderPositions;
        lastEncoderVelocities = lastTrackingEncoderVelocities;

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, Constants.Drivetrain.leftFrontHardwareMapName));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, Constants.Drivetrain.rightRearHardwareMapName));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, Constants.Drivetrain.rightFrontHardwareMapName));

        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int frontPos = frontEncoder.getCurrentPosition();

        lastEncoderPositions.clear();
        lastEncoderPositions.add(leftPos);
        lastEncoderPositions.add(rightPos);
        lastEncoderPositions.add(frontPos);

        return Arrays.asList(
                encoderTicksToInches(leftPos),
                encoderTicksToInches(rightPos),
                encoderTicksToInches(frontPos)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        int leftVel = (int) leftEncoder.getCorrectedVelocity();
        int rightVel = (int) rightEncoder.getCorrectedVelocity();
        int frontVel = (int) frontEncoder.getCorrectedVelocity();

        lastEncoderVelocities.clear();
        lastEncoderVelocities.add(leftVel);
        lastEncoderVelocities.add(rightVel);
        lastEncoderVelocities.add(frontVel);

        return Arrays.asList(
                encoderTicksToInches(leftVel),
                encoderTicksToInches(rightVel),
                encoderTicksToInches(frontVel)
        );
    }
}