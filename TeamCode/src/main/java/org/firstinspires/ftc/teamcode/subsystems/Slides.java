package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {
    DcMotor slideMotorLeft;
    DcMotor slideMotorRight;

    public Slides(HardwareMap hardwareMap) {
        slideMotorRight = hardwareMap.get(DcMotor.class, "rightSlide");
        slideMotorLeft = hardwareMap.get(DcMotor.class, "leftSlide");
        //sets max power
        slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorRight.setPower(1);
        slideMotorLeft.setPower(1);

        slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //sets both motors to go to targetPos
    public void setPosition(int targetPos) {
        slideMotorRight.setTargetPosition(targetPos);
        slideMotorLeft.setTargetPosition(targetPos);
    }
    public double getPosition(){
        return slideMotorLeft.getCurrentPosition();
    }

    //set manual movement
    public void manualPosition(double manualPower) {
        int newPos;
        int manualChange = (int) (manualPower * 85);
        newPos = slideMotorLeft.getTargetPosition() + manualChange;

        slideMotorRight.setTargetPosition(newPos);
        slideMotorLeft.setTargetPosition(newPos);
    }

    public void stopSlides() {
        slideMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotorRight.setPower(0);
        slideMotorLeft.setPower(0);
    }
}
