package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Intake
{
    DcMotor intakeMotor;

    Telemetry telemetry;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
    }

    public void setIntakePower(double intakePower) {
        intakeMotor.setPower(intakePower);
    }

    public void stopDrive() {
        intakeMotor.setPower(0);
        telemetry.addLine("stopped");
    }

}