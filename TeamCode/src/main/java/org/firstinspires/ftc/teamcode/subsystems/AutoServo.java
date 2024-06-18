package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class AutoServo {
    public Servo AutoServoL;
    public Servo AutoServoR;

    public AutoServo(HardwareMap hardwareMap) {
//        AutoServoR = hardwareMap.get(Servo.class, "handServoR");
//        AutoServoL = hardwareMap.get(Servo.class, "handServoL");

//        AutoServoR.setDirection(Servo.Direction.REVERSE);
//        AutoServoL.setDirection(Servo.Direction.FORWARD);
//        upBoth();
    }

    //Pre-sets
    public void DropLeft() {
//        AutoServoL.setPosition(Constants.AutoServo.downLeft);
//        AutoServoR.setPosition(Constants.AutoServo.upRight);
    }
    public void DropRight() {
//        AutoServoR.setPosition(Constants.AutoServo.downRight);
//        AutoServoL.setPosition(Constants.AutoServo.upLeft);
    }

    public void upBoth(){
//        AutoServoL.setPosition(Constants.AutoServo.upLeft);
//        AutoServoR.setPosition(Constants.AutoServo.upRight);
    }

    public void setAutoServoL(double pos) {
//        AutoServoL.setPosition(pos);
    }

    public void setAutoServoR(double pos) {
//        AutoServoR.setPosition(pos);
    }
}
