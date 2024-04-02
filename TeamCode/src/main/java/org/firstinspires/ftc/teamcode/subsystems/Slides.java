package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class Slides {
    boolean overwrite = false;
    DcMotorEx slideMotor;
    double manualChangeD; //DEZZ NUTS OHHHHHHHHHHHHHHH
    int manualChangeI;

    public Slides(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotorEx.class, "slide");
        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(0);

        //sets max power
        slideMotor.setPower(1);

        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor.setDirection(DcMotorEx.Direction.REVERSE);
        slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
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
    public void manualPosition(double manualPower, boolean tempOverwrite) {
        overwrite = tempOverwrite;
        manualChangeD = manualPower * Constants.Slides.manualSlidePower;
        manualChangeI = (int) manualChangeD;
        setPosition(getTargetPosition() + manualChangeI);
    }

    public void stopSlides() {
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotor.setPower(0);
    }

    public double getCurrent(CurrentUnit currentUnit) {

        return slideMotor.getCurrent(currentUnit);
    }

    public void slidesPeriodic() {
        if (getTargetPosition() > Constants.Slides.slideMax) {
            setPosition(Constants.Slides.slideMax);
        }
        if ((getTargetPosition() < Constants.Slides.slideMin)&& !overwrite) {
            setPosition(Constants.Slides.slideMin);
        }
    }
}
