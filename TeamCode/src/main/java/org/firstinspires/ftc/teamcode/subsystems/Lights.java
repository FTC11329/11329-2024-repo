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
    RevBlinkinLedDriver smartLEDLeft;
    RevBlinkinLedDriver smartLEDRight;
    DcMotor dumbLights;

    ElapsedTime elapsedTime = new ElapsedTime();


    public Lights(HardwareMap hardwareMap) {
        smartLEDLeft = hardwareMap.get(RevBlinkinLedDriver.class, "sLedL");
        smartLEDLeft = hardwareMap.get(RevBlinkinLedDriver.class, "sLedR");

        dumbLights = hardwareMap.get(DcMotor.class, "dumbLed");
        dumbLights.setDirection(DcMotorSimple.Direction.REVERSE);
        dumbLights.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Smart stuff
    public void setPatternLeft(RevBlinkinLedDriver.BlinkinPattern blinkinPattern) {
        smartLEDLeft.setPattern(blinkinPattern);
    }
    public void setPatternRight(RevBlinkinLedDriver.BlinkinPattern blinkinPattern) {
        smartLEDRight.setPattern(blinkinPattern);
    }

    //Dumb dump
    public void setDumbLed(double power) {
        dumbLights.setPower(power);
    }
    public void setDumbWave(double speed) {
        //Fancy math for wave
        dumbLights.setPower(-0.5 * (Math.sin(speed * elapsedTime.seconds())) + 0.5);
    }

    public void stopLights() {
        dumbLights.setPower(0);
    }
}