package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.ArrayList;

/**
 * Subsystem for controlling rev blinkin led
 */
public class Lights {
    DcMotor dumbLights;
    ElapsedTime elapsedTime = new ElapsedTime();
    LightWaveThread lightWaveThread;



    public Lights(HardwareMap hardwareMap) {
        dumbLights = hardwareMap.get(DcMotor.class, "dumbLed");
        dumbLights.setDirection(DcMotorSimple.Direction.REVERSE);
        dumbLights.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lightWaveThread = new LightWaveThread(this);
        lightWaveThread.start();
    }

    //Dumb dump
    public void setDumbLed(double power) {
        dumbLights.setPower(power);
    }
    public void setDumbFlash(double time) {
        if (elapsedTime.seconds() <= time) {
            setDumbLed(0.5);
        } else if (elapsedTime.seconds() >= time * 2) {
            elapsedTime.reset();
        } else {
            setDumbLed(0);
        }
    }
    public void startDumbWaveThread() {
        lightWaveThread.setRunning(true);
    }
    public void killDumbWaveThread() {
        lightWaveThread.setStopped();
    }
    public void stopDumbWaveThread() {
        lightWaveThread.setRunning(false);
    }

    public void setDumbWave(double speed, double min, double max) {
        //Fancy math for wave
        max = max / 2;
        min = min + max;
        speed = speed * 2 * Math.PI;
        setDumbLed(-max * (Math.cos(speed * elapsedTime.seconds())) + min);
    }

    public void stopLights() {
        dumbLights.setPower(0);
    }
}
class LightWaveThread extends Thread{
    volatile Lights lights;
    private boolean isRunning;
    private boolean stopped;
    public LightWaveThread(Lights lights) {
        super();
        this.lights = lights;
        isRunning = false;
        stopped = false;
    }

    @Override
    public void run() {
        while (isRunning && !stopped) {
            lights.setDumbWave(1,0, 1);
        }
    }

    public void setRunning(boolean isRunning) {
        this.isRunning = isRunning;
    }

    public void setStopped() {
        stopped = true;
    }
}