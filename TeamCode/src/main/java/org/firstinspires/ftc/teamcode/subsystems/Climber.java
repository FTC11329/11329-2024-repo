package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Climber {
    DcMotor climberMotor;

    public Climber(HardwareMap hardwareMap) {
        climberMotor = hardwareMap.get(DcMotor.class, "climber");
        climberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climberMotor.setTargetPosition(0);

        //sets max power
        climberMotor.setPower(1);

        climberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climberMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        climberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPos(int pos) {
        climberMotor.setTargetPosition(pos);
    }

    public int getPosition() {
        return climberMotor.getCurrentPosition();
    }

    public void stopClimber() {
        climberMotor.setPower(0);
    }
}
