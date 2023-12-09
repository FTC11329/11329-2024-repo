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
        armServoL = hardwareMap.get(Servo.class, "leftArmServo");
        armServoR = hardwareMap.get(Servo.class, "rightArmServo");

        setPosition(0);
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
        currentPos += (armPower * 0.01);
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
        if (isArmScared(slidesPos)) {
            setPosition(Constants.Arm.placePos);
        }
    }

    public boolean isArmScared(int slidesPos) {
        //if the arm could get pinched aka where it is not supposed to go
        return !(slidesPos < 10 || getCurrentPos() > Constants.Arm.scaredPos);
    }
}
