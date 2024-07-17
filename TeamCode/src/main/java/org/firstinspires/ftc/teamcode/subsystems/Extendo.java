package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.exception.NotANumberException;
import org.firstinspires.ftc.teamcode.Constants;

public class Extendo {
    public Servo extendo;

    boolean doesExtend = false;

    public Extendo(HardwareMap hardwareMap) {
        extendo = hardwareMap.get(Servo.class, "extendo");

        extendo.setDirection(Servo.Direction.REVERSE);

        setExtendo(Constants.Extendo.closed);
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

    double a = 106.733; //mm
    double c = 170.940; //mm
    public double rotationToMM(double rot) { //not works
        double radians = ((rot * 270 * 3 * Math.PI)/(4 * 180));
        //solving law of cos
        return Math.acos(radians) + (Math.sqrt(Math.pow(a,2) * Math.pow(Math.cos(radians), 2) - Math.pow(a, 2) + Math.pow(c, 2)));
    }

    //minimum 0, max 210
    public double mmToRotation(double mm) {
        mm *= 1.1; //offset
        mm += 64.2071 - /*arm*/ 75.7 + 50 /*offset*/;
        double radians = Math.PI - (Math.acos((Math.pow(a, 2) + Math.pow(mm, 2) - Math.pow(c, 2) ) / (2 * a * mm)));

        Double rot = (radians * 4 * 180 * 0.46)/(270 * 3 * Math.PI) * 1.23;
        if (rot.isNaN()) {
            return Constants.Extendo.extended;
        } else if (rot < Constants.Extendo.closed) {
            return Constants.Extendo.closed;
        } else if (rot > Constants.Extendo.extended) {
            return Constants.Extendo.extended;
        }
        return rot * 1.2;
    }

    public double inToRotation(double in) {
        return mmToRotation(in * 25.4);
    }

    public void periodic(double distanceToBd, boolean extendToBd) {
        if (extendToBd) {
            setExtendo(inToRotation(distanceToBd));
        }
    }
}
