package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Lights;

@TeleOp(name = "Lights Demo", group = "Allen op mode")
public class LightsTest extends OpMode {
    Lights lights;

    @Override
    public void init() {
        lights = new Lights(hardwareMap);

    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            lights.setDumbWave(7);
        } else {
            lights.setDumbFlash(0.4);
        }
    }

    @Override
    public void stop() {
        lights.stopLights();
    }
}
