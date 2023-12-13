package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Outtake {
    public Arm arm;
    public Slides slides;

    boolean upDebounce;
    public Outtake(HardwareMap hardwareMap) {
        arm = new Arm(hardwareMap);
        slides = new Slides(hardwareMap);
    }

    public void periodic() {
        arm.periodic(slides.getPosition());
    }

    public void manualSlides(double manualPower) {
        int manualChange = (int) (manualPower * Constants.Slides.manualSlidePower);
        slides.manualPosition(manualChange);
    }
    public void presetSlides(int slidesPos) {
        slides.setPosition(slidesPos);
    }
    public void upSlide(int upAmount, boolean go) {
        if (go && !upDebounce){
            manualSlides(upAmount);
            upDebounce = true;
        } else if (!go) {
            upDebounce = false;
        }
    }

    public int getSlidePosition() {
        return slides.getPosition();
    }
    public int getSlideTargetPosition() {
        return slides.getTargetPosition();
    }

    public void manualArm(double manualPower) {
        arm.manualPosition(manualPower);
    }
    public void presetArm(int armPos) {
        arm.setPosition(armPos);
    }
    public double getArmPosition() {
        return arm.getCurrentPos();
    }

    public void preset(int slidesPos, double armPos) {
        slides.setPosition(slidesPos);
        arm.setPosition(armPos);
    }

    public void stopOuttake() {
        slides.stopSlides();
    }
}