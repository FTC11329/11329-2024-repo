package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Claw {
    public Servo frontClawServo;
    public Servo backClawServo;

    public Servo wristServo;

    public Claw(HardwareMap hardwareMap) {
        frontClawServo = hardwareMap.get(Servo.class, "clawServoF");
        backClawServo = hardwareMap.get(Servo.class, "clawServoB");

        frontClawServo.setDirection(Servo.Direction.REVERSE);

        wristServo = hardwareMap.get(Servo.class, "wristClawServo");
        setWristPosition(1.0/3.0);
        setOpen(false);
    }

    public void setFrontHold(boolean hold) {
        if (hold) {
            frontClawServo.setPosition(Constants.Claw.frontClawHold);
        } else {
            frontClawServo.setPosition(Constants.Claw.frontClawDrop);
        }
    }

    public void setBackHold(boolean hold) {
        if (hold) {
            backClawServo.setPosition(Constants.Claw.backClawHold);
        } else {
            backClawServo.setPosition(Constants.Claw.backClawDrop);
        }
    }

    public void setOpen(boolean open) {
        setFrontHold(open);
        setBackHold(open);
    }

    public void setWristPosition(double pos) {
        wristServo.setPosition(pos);
    }


    //temp
    public void setFrontClawServo(double pos) {
        frontClawServo.setPosition(pos);
    }
    public void setBackClawServo(double pos) {
        backClawServo.setPosition(pos);
    }

    public void periodic() {
    }
    public void stopClaw() {
    }
}
