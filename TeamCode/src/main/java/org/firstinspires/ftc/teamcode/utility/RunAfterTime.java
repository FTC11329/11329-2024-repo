package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.util.ElapsedTime;


public class RunAfterTime {
    public static void runAfterTime(int timeElapsed, AnyFunction function) {
        ElapsedTime elapsedTime = new ElapsedTime();
        double startingTime = elapsedTime.milliseconds();
        while (elapsedTime.milliseconds() - startingTime < timeElapsed) {
        }
        function.run();
    }
}
