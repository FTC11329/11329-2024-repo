package org.firstinspires.ftc.teamcode.roadrunner.drive;


import org.firstinspires.ftc.teamcode.Constants;

/**
 * Please continue to use Constants.Roadrunner.* instead of importing each individual variable
 * This makes sure we don't get name collisions with other classes which have similar variables
 */
public class DriveUtilityMethods {

    public static double encoderTicksToInches(double ticks) {
        return Constants.Roadrunner.WHEEL_RADIUS * 2 * Math.PI * Constants.Roadrunner.GEAR_RATIO * ticks / Constants.Roadrunner.TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * Constants.Roadrunner.GEAR_RATIO * 2 * Math.PI * Constants.Roadrunner.WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
