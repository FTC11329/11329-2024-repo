package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Intake {
    Servo dropServo;
    DcMotor intakeMotor;
    Telemetry telemetry;

    public Intake(HardwareMap hardwareMap) {
//        dropServo = hardwareMap.get(Servo.class, "dropServo");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setDropServoPos(double servoPos){
        dropServo.setPosition(servoPos);
    }

    public void setIntakePower(double intakePower) {
        intakeMotor.setPower(intakePower);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
        telemetry.addLine("stopped");
    }

}