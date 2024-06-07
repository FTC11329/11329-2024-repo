package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Arm {
    public Servo armServoR;
    public Servo armServoL;

    public int slidesPos;
    public double currentPos;


    public Arm(HardwareMap hardwareMap) {
        armServoL = hardwareMap.get(Servo.class, "leftArmServo");
        armServoR = hardwareMap.get(Servo.class, "rightArmServo");

        armServoL.setDirection(Servo.Direction.FORWARD);
        armServoR.setDirection(Servo.Direction.FORWARD);
        setPosition(Constants.Arm.intakePos);
    }

    public double getCurrentPos() {
        return armServoR.getPosition();
    }

    public void setPosition(double armPos) {
            armServoL.setPosition(1.0 - armPos);
            armServoR.setPosition(armPos);
            currentPos = armPos;
    }

    public void manualPosition(double armPower) {
        currentPos += (armPower * Constants.Arm.manualPosition);
        if (currentPos > Constants.Arm.armMax) {
            currentPos = Constants.Arm.armMax;
        }
        if (currentPos < Constants.Arm.armMin) {
            currentPos = Constants.Arm.armMin;
        }
        armServoL.setPosition(1.0 - currentPos);
        armServoR.setPosition(currentPos);
    }

    public void periodic(int tempSlidesPos) {
        slidesPos = tempSlidesPos;
    }
}
