package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Plane {
    public Servo planeServo;

    public Plane(HardwareMap hardwareMap) {
        planeServo = hardwareMap.get(Servo.class, "planeServo");
        planeServo.setPosition(0);
    }

    public void setPos(double planePos) {
        planeServo.setPosition(planePos);
    }
}
