package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utility.BarcodePosition;

public class BackDistanceSensors {
    private com.qualcomm.robotcore.hardware.DistanceSensor BLSensor;
    private com.qualcomm.robotcore.hardware.DistanceSensor BRSensor;

    BarcodePosition spikeNumber;

    public BackDistanceSensors(HardwareMap hardwareMap) {
        BLSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "BLDistance");
        BRSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "BRDistance");
    }
    public double getBLeftState() {
        return BLSensor.getDistance(DistanceUnit.INCH);
    }

    public double getBRightState() {
        return BRSensor.getDistance(DistanceUnit.INCH);
    }
}
