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

public class ClawSensor {
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
    public Vector3D getFrontColor() {
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

    public Vector3D getBackColor() {
        colorBack = new Vector3D(colorSensorBack.red(), colorSensorBack.green(), colorSensorBack.blue());
        return colorBack;
    }
    public int getBackAlpha(){
        return colorSensorBack.alpha();
    }
    public double getBackDistance(DistanceUnit distanceUnit) {
        return colorSensorBack.getDistance(distanceUnit);
    }

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
}
