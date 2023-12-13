package org.firstinspires.ftc.teamcode.utility;

public class ControlUtils {
    public static double mapAxisToRange(double axis, double min, double max) {
        double percent = (axis + 1) / 2;
        return min + percent * (max - min);
    }
}
