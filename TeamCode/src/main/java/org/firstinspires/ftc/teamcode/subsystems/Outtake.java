package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

public class Outtake {
    private Arm arm;
    private Claw claw;
    public Slides slides;
    public Extendo extendo;
    public ClawSensor clawSensor;

    private int wristPos = 3;

    public Outtake(HardwareMap hardwareMap) {
        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);
        slides = new Slides(hardwareMap);
        extendo = new Extendo(hardwareMap);
        clawSensor = new ClawSensor(hardwareMap);
    }

    public void periodic() {
        arm.periodic(slides.getPosition());
        claw.periodic();
        slides.slidesPeriodic();
        if(getArmPosition() > Constants.Arm.safeArmPos) {
            if (wristPos <= 0) {
                wristPos = 7;
            } else if (wristPos >= 8) {
                wristPos = 1;
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
        if (slides.getPosition() > Constants.Slides.safeSlidePos) {
            arm.manualPosition(manualPower);
        }
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

    public void createPresetThread(int slidePos, double armPos, int wristPos, boolean extend, boolean grabbed, boolean goingUp) {
        double rot;
        if (extend) {
            rot = Constants.Extendo.extended;
        } else {
            rot = Constants.Extendo.closed;
        }
        new PresetThread(this, slidePos, armPos, wristPos, rot, grabbed, goingUp).start();
    }
    public void createPresetThread(int slidePos, double armPos, int wristPos, double extend, boolean goingUp) {
        new PresetThread(this, slidePos, armPos, wristPos, extend, false, goingUp).start();
    }
    public void createPresetThread(int slidePos, double armPos, int wristPos, boolean extend, boolean goingUp) {
        double rot;
        if (extend) {
            rot = Constants.Extendo.extended;
        } else {
            rot = Constants.Extendo.closed;
        }
        new PresetThread(this, slidePos, armPos, wristPos, rot, false, goingUp).start();
    }

    //Claw Sensor
    public boolean isFull() {
        return clawSensor.isFull();
    }
    public boolean isFrontSensor() {
        return clawSensor.isFrontDistance();
    }
    public boolean isBackSensor() {
        return clawSensor.isBackDistance();
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
    public int getTriedWristPos() {
        return wristPos;
    }
    public void manualWrist(int amount) {
        wristPos += amount;
    }
    //Extendo
    public void extend(boolean extend) {
        extendo.extend(extend);
    }
    public void setExtendo(double rot) {
        extendo.setExtendo(rot);
    }

    //stop it, get some help
    public void stopOuttake() {
        slides.stopSlides();
    }
}
//Allen's first thread! YIPIEEEEEEE
class PresetThread extends Thread{
    private volatile Outtake outtake;
    private int slidePos;
    private int wristPos;
    private double armPos;
    private double extend;
    private boolean goingUp;
    private boolean grabbed;

    public PresetThread(Outtake outtake, int slidePos, double armPos, int wristPos, double extend, boolean grabbed, boolean goingUp) {
        super();
        this.outtake = outtake;
        this.slidePos = slidePos;
        this.wristPos = wristPos;
        this.armPos = armPos;
        this.extend = extend;
        this.goingUp = goingUp;
        this.grabbed = grabbed;
    }

    @Override
    public void run() {
        if (goingUp) {
            if (!grabbed) {
                outtake.presetSlides(-20);
                outtake.holdClaw(true);
                try {
                    sleep(200);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            outtake.presetSlides(slidePos);
            while (outtake.getSlidePosition() < Constants.Slides.safeSlidePos) {
                try {
                    sleep(30);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            outtake.presetArm(armPos);
            outtake.setExtendo(extend);
            try {
                sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            outtake.setWristPos(wristPos);
        } else {
            outtake.presetArm(Constants.Arm.safeArmPos + 0.001);
            outtake.presetSlides(Constants.Slides.safeSlidePos + 100);
            outtake.setWristPos(wristPos);
            outtake.holdClaw(false);
            outtake.extend(false);
            try {
                sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            outtake.presetArm(armPos);
            outtake.holdClaw(false);
            try {
                sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            outtake.presetSlides(slidePos);
        }
    }
}