package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Subsystem for controlling rev blinkin led
 */
public class Lights {
    DcMotor dumbLights;

    ElapsedTime elapsedTime = new ElapsedTime();


    public Lights(HardwareMap hardwareMap) {
        dumbLights = hardwareMap.get(DcMotor.class, "dumbLed");
        dumbLights.setDirection(DcMotorSimple.Direction.REVERSE);
        dumbLights.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Dumb dump
    //IMPORTANT ----- YOU HAVE TO USE setDumbLed BECAUSE THE POWER HAS TO BE *-1???????
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