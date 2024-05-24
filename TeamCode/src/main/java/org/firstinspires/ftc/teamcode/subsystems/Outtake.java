package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

public class Outtake {
    private Arm arm;
    private Claw claw;
    public Slides slides;
    private ClawSensor clawSensor;

    public int wristPos = 3;

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
        if(getArmPosition() > 0.35) {
            if (wristPos <= 0) {
                wristPos = 1;
            } else if (wristPos >= 8) {
                wristPos = 7;
            }
            switch (wristPos) {
                case 1: {
                    claw.setWristPosition(Constants.Claw.wrist1);
                    break;
                }
                case 2: {
                    claw.setWristPosition(Constants.Claw.wrist2);
                    break;
                }
                case 3: {
                    claw.setWristPosition(Constants.Claw.wrist3);
                    break;
                }
                case 4: {
                    claw.setWristPosition(Constants.Claw.wrist4);
                    break;
                }
                case 5: {
                    claw.setWristPosition(Constants.Claw.wrist5);
                    break;
                }
                case 6: {
                    claw.setWristPosition(Constants.Claw.wrist6);
                    break;
                }
                case 7: {
                    claw.setWristPosition(Constants.Claw.wrist7);
                    break;
                }
            }
        } else {
            claw.setWristPosition(Constants.Claw.wrist3);
        }
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
        presetSlides(slidesPos);
        presetArm(armPos);
    }
    public void createPresetThread(int slidePos, double armPos, int wristPos, boolean goingUp) {
        new PresetThread(this, slidePos, armPos, wristPos, goingUp).start();
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
    public void setWristPos(int pos) {
        wristPos = pos;
    }
    public void manualWrist(int amount) {
        wristPos += amount;
    }

    //stop it, get some help
    public void stopOuttake() {
        slides.stopSlides();
    }

    //temp
    public int getthing() {
        return wristPos;
    }
}
//Allen's first thread! YIPIEEEEEEE
class PresetThread extends Thread{
    ElapsedTime time = new ElapsedTime();
    double time1 = 0;
    private volatile Outtake outtake;
    private int slidePos;
    private int wristPos;
    private double armPos;
    private boolean goingUp;

    public PresetThread(Outtake outtake, int slidePos, double armPos, int wristPos, boolean goingUp) {
        super();
        this.outtake = outtake;
        this.slidePos = slidePos;
        this.wristPos = wristPos;
        this.armPos = armPos;
        this.goingUp = goingUp;
    }

    @Override
    public void run() {
        if(goingUp) {
            outtake.presetSlides(slidePos);
            while (outtake.getSlidePosition() < Constants.Slides.safeSlidePos) {
                try {
                    sleep(30);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            if (outtake.getArmPosition() > Constants.Arm.safeArmPos) {
                outtake.presetArm(armPos);
                outtake.wristPos = wristPos;
                return;
            } else {
                outtake.presetArm(armPos);
            }
            try {
                sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            outtake.wristPos = wristPos;
        }
    }
}