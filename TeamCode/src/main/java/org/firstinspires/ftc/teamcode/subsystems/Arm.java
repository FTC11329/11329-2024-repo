package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Arm {
    public Servo armServoR;
    public Servo armServoL;

    public int slidesPos;
    public double targetPos;
    public double currentPos;

    public Boolean sadArm = false;


    public Arm(HardwareMap hardwareMap) {
        armServoR = hardwareMap.get(Servo.class, "rightArmServo");
        armServoL = hardwareMap.get(Servo.class, "leftArmServo");

        setPosition(0, slidesPos);
    }

    public void setPosition(double armPos, int slidesPos) {
        if (!isArmPinched(slidesPos, currentPos)) {
            armServoL.setPosition(1.0 - armPos);
            armServoR.setPosition(armPos);
            currentPos = armPos;
        }
    }

    public void manualPosition(double armPower) {
        currentPos += (armPower * 0.01);
        armServoL.setPosition(1.0 - currentPos);
        armServoR.setPosition(currentPos);
    }

    public void periodic(int tempSlidesPos) {
        slidesPos = tempSlidesPos;
        if (isArmPinched(slidesPos, currentPos)) {
            setPosition(Constants.Arm.safePos, slidesPos);
            sadArm = true;
        } else if (!sadArm) {
            targetPos = currentPos;
        } else {
            armServoL.setPosition(1.0 - targetPos);
            armServoR.setPosition(targetPos);
            sadArm = false;
        }
    }

    public boolean isArmPinched (int slidesPos, double currentPos) {
        //if the arm could get pinched aka where it is not supposed to go
        return (currentPos < Constants.Arm.groundPinchMax && currentPos > Constants.Arm.groundPinchMin && slidesPos < Constants.Slides.groundPinchMax && slidesPos > Constants.Slides.groundPinchMin);
    }
}
