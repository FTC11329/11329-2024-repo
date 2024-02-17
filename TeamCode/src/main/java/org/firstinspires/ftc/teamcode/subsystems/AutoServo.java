package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class AutoServo {
    public Servo AutoServoL;
    public Servo AutoServoR;

    public AutoServo(HardwareMap hardwareMap) {
        AutoServoR = hardwareMap.get(Servo.class, "handServoR");
        AutoServoL = hardwareMap.get(Servo.class, "handServoL");

        AutoServoR.setDirection(Servo.Direction.REVERSE);
        AutoServoL.setDirection(Servo.Direction.FORWARD);
        setAutoServoL(0);
        setAutoServoR(0);
    }
    //Pre-sets
    public void DropLeft() {
        AutoServoR.setPosition(Constants.AutoServo.downRight);
    }
    public void DropRight() {
        AutoServoL.setPosition(Constants.AutoServo.downLeft);
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
