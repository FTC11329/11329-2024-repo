package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

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
        if (getSlidePosition() > 500) {
            arm.setPosition(armPos);
        }
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
        presetSlides(slidesPos);
        presetArm(armPos);
    }

    //Claw Sensor
    public boolean isFull() {
        return clawSensor.isFull();
    }
    public boolean isEmpty() {
        return clawSensor.isEmpty();
    }

    //Claw
    public void holdClaw(boolean hold) {
        claw.setOpen(hold);
    }
    public void holdFrontClaw(boolean hold) {
        claw.setFrontHold(hold);
    }
    public void holdBackClaw(boolean hold) {
        claw.setBackHold(hold);
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