package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slides {
    DcMotor slideMotorLeft;
    DcMotor slideMotorRight;

    public Slides(HardwareMap hardwareMap) {
        slideMotorRight = hardwareMap.get(DcMotor.class, "rightSlide");
        slideMotorLeft  = hardwareMap.get(DcMotor.class, "leftSlide");
        //sets max power
        slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorRight.setPower(1);
        slideMotorLeft .setPower(1);

        slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorLeft .setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotorLeft .setDirection(DcMotorSimple.Direction.FORWARD);
    }
    //sets both motors to go to targetPos
    public void toPosition(int targetPos) {
        slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorLeft .setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorRight.setPower(1);
        slideMotorRight.setPower(1);
        slideMotorRight.setTargetPosition(targetPos);
        slideMotorLeft .setTargetPosition(targetPos);
    }
}
