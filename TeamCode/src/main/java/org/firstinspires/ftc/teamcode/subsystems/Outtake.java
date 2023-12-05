package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {
    public Arm arm;
    public Slides slides;

    public Outtake(HardwareMap hardwareMap) {
        arm = new Arm(hardwareMap);
        slides = new Slides(hardwareMap);
    }

    public void periodic() {
        arm.periodic(slides.getPosition());
    }

    public void manualSlides(double manualPower) {
        slides.manualPosition(manualPower);
    }

    public void manualArm(double manualPower) {
        arm.manualPosition(manualPower);
    }

    public void presetSlides(int slidesPos) {
        slides.setPosition(slidesPos);
    }

    public void presetArm(int armPos) {
        arm.setPosition(armPos);
    }

    public void preset(int slidesPos, int armPos) {
        slides.setPosition(slidesPos);
        arm.setPosition(armPos, slides.getPosition());
    }

    public void stop() {
        slides.stopSlides();
    }
}
