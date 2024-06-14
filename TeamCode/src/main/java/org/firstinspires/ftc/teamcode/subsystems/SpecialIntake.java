package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;


public class SpecialIntake {
    Servo intakeServo;

    public SpecialIntake(HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(Servo.class, "specialIntake");
        intakeServo.setPosition(0);
    }

    public void setIntakeServo(double servoPos) {
        if (Constants.SpecialIntake.max >= servoPos && servoPos >= Constants.SpecialIntake.min) {
            intakeServo.setPosition(servoPos);
        }
    }

    public void manualHeight(double powerTemp) {
        double power = powerTemp * Constants.SpecialIntake.manualPosition;
        setIntakeServo(power);
    }
}