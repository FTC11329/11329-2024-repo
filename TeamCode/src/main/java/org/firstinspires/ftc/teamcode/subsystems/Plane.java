package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Plane {
    public Servo planeServo;

    public Plane(HardwareMap hardwareMap) {
        planeServo = hardwareMap.get(Servo.class, Constants.Plane.servoName);
        planeServo.setDirection(Servo.Direction.REVERSE);
        planeServo.setPosition(0.1);
    }

    public void setPos(double planePos) {
        planeServo.setPosition(planePos);
    }

    public void fire() {
        planeServo.setPosition(Constants.Plane.fire);
    }

    public void hold() {
        planeServo.setPosition(Constants.Plane.hold);
    }
}
