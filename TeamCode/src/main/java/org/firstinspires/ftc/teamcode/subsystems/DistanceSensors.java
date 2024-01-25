package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class DistanceSensors {
    private com.qualcomm.robotcore.hardware.DistanceSensor leftDistanceSensor;
    private com.qualcomm.robotcore.hardware.DistanceSensor rightDistanceSensor;

    int spikeNumber = 0;

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



    public int getDirectionBlue() {
        if (leftDistanceSensor.getDistance(DistanceUnit.INCH) < Constants.DistanceSensors.Tolerance) {
            spikeNumber = 3; //left
        } else if (rightDistanceSensor.getDistance(DistanceUnit.INCH) < Constants.DistanceSensors.Tolerance){
            spikeNumber = 2; //center
        } else {
            spikeNumber = 1; //right
        }
        return spikeNumber;
    }
    public int getDirectionRed() {
        if (leftDistanceSensor.getDistance(DistanceUnit.INCH) < Constants.DistanceSensors.Tolerance) {
            spikeNumber = 1; //left
        } else if (rightDistanceSensor.getDistance(DistanceUnit.INCH) < Constants.DistanceSensors.Tolerance){
            spikeNumber = 2; //center
        } else {
            spikeNumber = 3; //right
        }
        return spikeNumber;
    }
}
