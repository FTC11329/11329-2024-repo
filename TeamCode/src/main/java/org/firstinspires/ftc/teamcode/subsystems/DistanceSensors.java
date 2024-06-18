package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utility.BarcodePosition;

public class DistanceSensors {
    private com.qualcomm.robotcore.hardware.DistanceSensor leftDistanceSensor;
    private com.qualcomm.robotcore.hardware.DistanceSensor rightDistanceSensor;

    BarcodePosition spikeNumber;

    public DistanceSensors(HardwareMap hardwareMap) {
        leftDistanceSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "left_distance");
        rightDistanceSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "right_distance");
    }
    public double getLeftState() {
        return leftDistanceSensor.getDistance(DistanceUnit.INCH);
    }

    public double getRightState() {
        return rightDistanceSensor.getDistance(DistanceUnit.INCH);
    }



    public BarcodePosition getDirectionBlue(boolean right) {
        if (right) {
            if (leftDistanceSensor.getDistance(DistanceUnit.INCH) < Constants.DistanceSensors.longTolerance) {
                spikeNumber = BarcodePosition.Two; //left
            } else if (rightDistanceSensor.getDistance(DistanceUnit.INCH) < Constants.DistanceSensors.shortTolerance) {
                spikeNumber = BarcodePosition.Three; //center
            } else {
                spikeNumber = BarcodePosition.One; //right
            }
        } else {
            if (leftDistanceSensor.getDistance(DistanceUnit.INCH) < Constants.DistanceSensors.shortTolerance) {
                spikeNumber = BarcodePosition.One; //left
            } else if (rightDistanceSensor.getDistance(DistanceUnit.INCH) < Constants.DistanceSensors.longTolerance) {
                spikeNumber = BarcodePosition.Two; //center
            } else {
                spikeNumber = BarcodePosition.Three; //right
            }
        }
        return spikeNumber;
    }


    public BarcodePosition getDirectionRed(boolean left) {
        if (left) {
            if (leftDistanceSensor.getDistance(DistanceUnit.INCH) < Constants.DistanceSensors.longTolerance) {
                spikeNumber = BarcodePosition.One; //left
            } else if (rightDistanceSensor.getDistance(DistanceUnit.INCH) < Constants.DistanceSensors.shortTolerance) {
                spikeNumber = BarcodePosition.Two; //center
            } else {
                spikeNumber = BarcodePosition.Three; //right
            }
        } else {
            if (leftDistanceSensor.getDistance(DistanceUnit.INCH) < Constants.DistanceSensors.shortTolerance) {
                spikeNumber = BarcodePosition.Two; //left
            } else if (rightDistanceSensor.getDistance(DistanceUnit.INCH) < Constants.DistanceSensors.longTolerance) {
                spikeNumber = BarcodePosition.Three; //center
            } else {
                spikeNumber = BarcodePosition.One; //right
            }
        }
        return spikeNumber;
    }
}
