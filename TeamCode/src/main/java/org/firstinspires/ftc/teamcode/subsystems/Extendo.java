package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Extendo {
    public Servo extendo;

    boolean doesExtend = false;

    public Extendo(HardwareMap hardwareMap) {
        extendo = hardwareMap.get(Servo.class, "extendo");

        extendo.setDirection(Servo.Direction.REVERSE);

        setExtendo(0);
    }

    public void setExtendo(double pos) {
        extendo.setPosition(pos);
    }

    public void extend(boolean extend) {
        if (extend) {
            extendo.setPosition(Constants.Extendo.extended);
        } else {
            extendo.setPosition(Constants.Extendo.closed);
        }
    }

    public void distanceExtend(boolean doesExtend) {
        this.doesExtend = doesExtend;
    }

    double a = 106.733;
    double c = 170.940;
    public double rotationToMM(double rot) {
        double radians = ((rot * 270 * 3 * Math.PI)/(4 * 180));
        //solving law of cos
        return Math.acos(radians) + (Math.sqrt(Math.pow(a,2) * Math.pow(Math.cos(radians), 2) - Math.pow(a, 2) + Math.pow(c, 2)));
    }

    public double mmToRotation(double mm) {
        double radians = Math.acos((-Math.pow(c, 2) - Math.pow(a, 2) - Math.pow(mm, 2)) / (2 * a * mm)) + 2 * Math.PI;

        return (radians * 4 * 180)/(270 * 3 * Math.PI);

    }

    public void periodic() {
    }
}
