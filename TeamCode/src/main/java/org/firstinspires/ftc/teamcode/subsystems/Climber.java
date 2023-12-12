package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Climber {
    DcMotor climberMotor;

    public Climber(HardwareMap hardwareMap) {
        climberMotor = hardwareMap.get(DcMotor.class, "climber");
        climberMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climberMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        climberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        climberMotor.setPower(power);
    }

    //sets both motors to go to targetPos

    public int getPosition() {
        return climberMotor.getCurrentPosition();
    }

    public void stopClimber() {
        climberMotor.setPower(0);
    }
}
