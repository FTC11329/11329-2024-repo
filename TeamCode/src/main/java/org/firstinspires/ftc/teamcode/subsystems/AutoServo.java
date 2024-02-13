package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoServo {
    public Servo AutoServoL;
    public Servo AutoServoR;

    public AutoServo(HardwareMap hardwareMap) {
        AutoServoR = hardwareMap.get(Servo.class, "handServoR");
        AutoServoL = hardwareMap.get(Servo.class, "handServoL");

        AutoServoR.setDirection(Servo.Direction.FORWARD);
        AutoServoL.setDirection(Servo.Direction.REVERSE);
        setAutoServoL(0);
        setAutoServoR(0);
    }
    //Pre-sets
    public void DropLeft() {
        AutoServoR.setPosition(0);
    }
    public void DropRight() {
        AutoServoL.setPosition(0);
    }
    public void upBoth(){
        AutoServoL.setPosition(0);
        AutoServoR.setPosition(0);
    }

    public void setAutoServoL(double pos) {
        AutoServoL.setPosition(pos);
    }

    public void setAutoServoR(double pos) {
        AutoServoR.setPosition(pos);
    }
}
