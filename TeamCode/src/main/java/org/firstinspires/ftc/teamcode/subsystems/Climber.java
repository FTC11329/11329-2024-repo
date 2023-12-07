package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Climber {
    DcMotor slideMotor;

    public Climber(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotor.class, "climber");
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        slideMotor.setPower(power);
    }

    //sets both motors to go to targetPos

    public int getPosition() {
        return slideMotor.getCurrentPosition();
    }

    public void stopClimber() {
        slideMotor.setPower(0);
    }
}
