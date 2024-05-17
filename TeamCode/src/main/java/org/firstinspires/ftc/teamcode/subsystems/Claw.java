package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

        wristServo = hardwareMap.get(Servo.class, "wristClawServo");
    }

    public void setFrontOpen(boolean open) {
        if (open) {
            frontClawServo.setPosition(Constants.Claw.frontClawOpen);
        } else {
            frontClawServo.setPosition(Constants.Claw.frontClawClosed);
        }
    }

    public void setBackOpen(boolean open) {
        if (open) {
            backClawServo.setPosition(Constants.Claw.backClawOpen);
        } else {
            backClawServo.setPosition(Constants.Claw.backClawClosed);
        }
    }

    public void setOpen(boolean open) {
        setFrontOpen(open);
        setBackOpen(open);
    }

    public void setWristPos(double pos) {
        wristServo.setPosition(pos);
    }


    //temp
    public void setFrontClawServo(double pos) {
        frontClawServo.setPosition(pos);
    }

    public void periodic() {
    }
    public void stopClaw() {
    }
}
