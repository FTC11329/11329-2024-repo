package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {
    public CRServo clawServo;

    public Claw(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(CRServo.class, "clawServo");
    }

    public void setPower(double clawPower) {
        clawServo.setPower(clawPower);
    }

    public void stopClaw() {
        clawServo.setPower(0);
    }
}
