package org.firstinspires.ftc.teamcode.subsystems;

import static android.graphics.Color.argb;

import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utility.ColorEnum;

public class ClawSensor {
    boolean runInAuto = false;
    RevColorSensorV3 colorSensorFront;
    RevColorSensorV3 colorSensorBack;
    Vector3D colorFront;
    Vector3D colorBack;

    public ClawSensor(HardwareMap hardwareMap) {
        colorSensorFront = hardwareMap.get(RevColorSensorV3.class, "colorSensorF");
        colorSensorBack  = hardwareMap.get(RevColorSensorV3.class, "colorSensorB");

        colorSensorFront.enableLed(true);
        colorSensorBack .enableLed(true);
    }
    //colors sensors
    public Vector3D getFrontColorNumbers() {
        colorFront = new Vector3D(colorSensorFront.red(), colorSensorFront.green(), colorSensorFront.blue());
        return colorFront;
    }
    public int getFrontAlpha() {
        return colorSensorFront.alpha();
    }
    public double getFrontDistance(DistanceUnit distanceUnit) {
        if (colorSensorFront.getDistance(DistanceUnit.INCH) == 0) {
            return 10;
        } else {
            return colorSensorFront.getDistance(distanceUnit);
        }
    }

    public Vector3D getBackColorNumbers() {
        colorBack = new Vector3D(colorSensorBack.red(), colorSensorBack.green(), colorSensorBack.blue());
        return colorBack;
    }
    public int getBackAlpha(){
        return colorSensorBack.alpha();
    }
    public double getBackDistance(DistanceUnit distanceUnit) {
        return colorSensorBack.getDistance(distanceUnit);
    }

    //colors
    public ColorEnum getBackColor() {
        Vector3D color;
        double white;
        double yellow;
        double green;
        double purple;

        if (isBackDistance()) {
            color = getBackColorNumbers();

            white  = color.distance(Constants.ClawSensor.whiteBack);
            yellow = color.distance(Constants.ClawSensor.yellowBack);
            green  = color.distance(Constants.ClawSensor.greenBack);
            purple = color.distance(Constants.ClawSensor.purpleBack);

        } else {
            //!don't not no see nothing
            return ColorEnum.Empty;
        }

        if (white < yellow && white < green && white < purple) {
            return ColorEnum.White;
        } else if (yellow < white && yellow < green && yellow < purple) {
            return ColorEnum.Yellow;
        } else if (green < white && green < yellow && green < purple) {
            return ColorEnum.Green;
        } else if (purple < white && purple < yellow && purple < green) {
            return ColorEnum.Purple;
        }
        return ColorEnum.Empty;
    }
    public ColorEnum getFrontColor() {
        Vector3D color;
        double white;
        double yellow;
        double green;
        double purple;

        if (isFrontDistance()) {
            color = getFrontColorNumbers();

            white  = color.distance(Constants.ClawSensor.whiteFront);
            yellow = color.distance(Constants.ClawSensor.yellowFront);
            green  = color.distance(Constants.ClawSensor.greenFront);
            purple = color.distance(Constants.ClawSensor.purpleFront);

        } else {
            //!don't not no see nothing
            return ColorEnum.Empty;
        }

        if (white < yellow && white < green && white < purple) {
            return ColorEnum.White;
        } else if (yellow < white && yellow < green && yellow < purple) {
            return ColorEnum.Yellow;
        } else if (green < white && green < yellow && green < purple) {
            return ColorEnum.Green;
        } else if (purple < white && purple < yellow && purple < green) {
            return ColorEnum.Purple;
        }
        return ColorEnum.Empty;
    }


    //Distances
    public boolean isFrontDistance() {
        return (colorSensorFront.getDistance(DistanceUnit.INCH) <= Constants.ClawSensor.distanceLimit);
    }
    public boolean isBackDistance() {
        return (colorSensorBack.getDistance(DistanceUnit.INCH) <= Constants.ClawSensor.distanceLimit);
    }

    public boolean isEmpty() {
        return !(isFrontDistance() || isBackDistance());
    }
    public boolean isFull() {
        return (isFrontDistance() && isBackDistance());
    }

    //Auto
    public void setRunInAuto(boolean run) {
        runInAuto = run;
    }

    public boolean autoSense() {
        if (runInAuto) {
            return isFull();
        } else {
            return false;
        }
    }
}
