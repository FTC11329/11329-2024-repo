package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Outtake {
    public Arm arm;
    public Claw claw;
    public Slides slides;
    public ClawSensor clawSensor;


    public Outtake(HardwareMap hardwareMap) {
        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);
        slides = new Slides(hardwareMap);
        clawSensor = new ClawSensor(hardwareMap);
    }

    public void periodic() {
        arm.periodic(slides.getPosition());
        claw.periodic();
        slides.slidesPeriodic();
    }

    //Arm
    public void manualArm(double manualPower) {
        arm.manualPosition(manualPower);
    }
    public void presetArm(double armPos) {
        arm.setPosition(armPos);
    }
    public double getArmPosition() {
        return arm.getCurrentPos();
    }

    //Slides
    public void manualSlides(double manualPower, boolean overwrite) {
        slides.manualPosition(manualPower, overwrite);
    }
    public void presetSlides(int slidesPos) {
        slides.setPosition(slidesPos);
    }
    public int getSlidePosition() {
        return slides.getPosition();
    }
    public int getSlideTargetPosition() {
        return slides.getTargetPosition();
    }

    public void preset(int slidesPos, double armPos) {
        slides.setPosition(slidesPos);
        arm.setPosition(armPos);
    }

    //Claw Sensor
    public boolean isFull() {
        return clawSensor.isFull();
    }
    public boolean isEmpty() {
        return clawSensor.isEmpty();
    }

    //Claw
    public void openClaw(boolean open) {
        claw.setOpen(open);
    }
    public void openFrontClaw(boolean open) {
        claw.setFrontOpen(open);
    }
    public void openBackClaw(boolean open) {
        claw.setBackOpen(open);
    }
    public void setWristPos(double pos) {
        claw.setWristPos(pos);
    }


    //stop it, get some help
    public void stopOuttake() {
        slides.stopSlides();
    }

    //temp
    public void setFrontClawServo(double pos) {
        claw.setFrontClawServo(pos);
    }
}