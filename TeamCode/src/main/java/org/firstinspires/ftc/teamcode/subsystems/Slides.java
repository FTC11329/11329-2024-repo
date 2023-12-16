package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;

public class Slides {
    DcMotor slideMotor;

    public Slides(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotor.class, "slide");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(0);

        //sets max power
        slideMotor.setPower(1);

        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        slideMotor.setPower(power);
    }

    //sets both motors to go to targetPos
    public void setPosition(int targetPos) {
        slideMotor.setTargetPosition(targetPos);
    }

    public int getPosition() {
        return slideMotor.getCurrentPosition();
    }

    public int getTargetPosition() {
        return slideMotor.getTargetPosition();
    }

    //set manual movement
    public void manualPosition(double manualPower) {
        int manualChange = (int) Math.round(manualPower * Constants.Slides.manualSlidePower);

        int newPos = getTargetPosition() + manualChange;

        newPos = Range.clip(newPos, Constants.Slides.slideMin, Constants.Slides.slideMax);

        slideMotor.setTargetPosition(newPos);
    }

    public void stopSlides() {
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotor.setPower(0);
    }
}
