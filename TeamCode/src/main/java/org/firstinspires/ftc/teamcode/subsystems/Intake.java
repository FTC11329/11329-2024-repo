package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Intake {
    DcMotor intakeMotor;

    Telemetry telemetry;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setIntakePower(double intakePower) {
        intakeMotor.setPower(intakePower);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
        telemetry.addLine("stopped");
    }

}