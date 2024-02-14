package org.firstinspires.ftc.teamcode.subsystems;

import static android.graphics.Color.argb;

import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class ClawSensor {
    ColorSensor colorSensorFront;
    ColorSensor colorSensorBack;
    Vector3D colorFront;
    Vector3D colorBack;

    public ClawSensor(HardwareMap hardwareMap) {
        colorSensorFront = hardwareMap.get(ColorSensor.class, "colorSensorF");
        colorSensorBack  = hardwareMap.get(ColorSensor.class, "colorSensorB");

        colorSensorFront.enableLed(true);
        colorSensorBack .enableLed(true);
    }
    public Vector3D getFrontColor() {
        colorFront = new Vector3D(colorSensorFront.red(), colorSensorFront.green(), colorSensorFront.blue());
        return colorFront;
    }
    public Vector3D getBackColor() {
        colorBack = new Vector3D(colorSensorBack.red(), colorSensorBack.green(), colorSensorBack.blue());
        return colorBack;
    }
}
