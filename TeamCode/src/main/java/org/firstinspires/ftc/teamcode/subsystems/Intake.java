package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Intake {
    DcMotor intakeMotor;
    CRServo intakeServo;
    Telemetry telemetry;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setIntakePower(double intakePower, int slidePos) {
        intakeMotor.setPower(intakePower);
    }

    public void setIntakeServoPower(double power) {
        intakeServo.setPower(power);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }
}