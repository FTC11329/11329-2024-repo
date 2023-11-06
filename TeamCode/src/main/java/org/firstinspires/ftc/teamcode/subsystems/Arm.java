package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Arm {
    public Servo armServoR;
    public Servo armServoL;
    public double targetPos;
    public double currentPos; //only tracked for manual control
    public boolean targeting;
    public Arm(HardwareMap hardwareMap) {
        armServoR = hardwareMap.get(Servo.class, "rightArmServo");
        armServoL = hardwareMap.get(Servo.class, "leftArmServo");
    }

    public void setPosition(double ArmPos){
        armServoL.setPosition(1.0 - ArmPos);
        armServoR.setPosition(ArmPos);
    }
    public void manualPosition(double armPower){
        currentPos += armPower;
        armServoL.setPosition(1.0 - currentPos);
        armServoR.setPosition(currentPos);
    }
    public void periodic(double slidesPos){
        if(targeting){
            if(!(currentPos < Constants.Arm.groundPinchMax && currentPos > Constants.Arm.groundPinchMin && slidesPos <Constants.Slides.groundPinchMax && slidesPos > Constants.Slides.groundPinchMin)){
                armServoL.setPosition(1.0 - targetPos);
                armServoR.setPosition(targetPos);
            }

        }
    }
    public void setPosition(double ArmPos, double slidesPos) {
        if(!(ArmPos < Constants.Arm.groundPinchMax && ArmPos > Constants.Arm.groundPinchMin && slidesPos <Constants.Slides.groundPinchMax && slidesPos > Constants.Slides.groundPinchMin)){
            armServoL.setPosition(1.0 - ArmPos);
            armServoR.setPosition(ArmPos);
            targeting = false;
            currentPos = ArmPos;
        }
        else{
            targeting = true;
            targetPos = ArmPos;
        }

    }
}
