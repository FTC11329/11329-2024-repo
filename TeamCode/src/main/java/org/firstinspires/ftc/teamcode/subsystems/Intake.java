package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Intake {
    DcMotor intakeMotor;
    Telemetry telemetry;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setIntakePower(double intakePower, int slidePos) {
        //if you can find a cleaner way to make sure the intake doesn't run with the slide up, please fix this.
//        if (slidePos > Constants.Slides.slideMin) {
        intakeMotor.setPower(intakePower);
//        }
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
        telemetry.addLine("stopped");
    }
}