package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {
    public CRServo clawServo;

    public Claw(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(CRServo.class, "clawServo");
        //1 is intake -1 is outtake
        clawServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double clawPower) {
        clawServo.setPower(clawPower);
    }

    public void stopClaw() {
        clawServo.setPower(0);
    }
}
