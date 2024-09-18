package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.DoubleSupplier;

public class OpticalOdometry extends Odometry {
    SparkFunOTOS myOtos;

    public OpticalOdometry(HardwareMap hardwareMap) {
        super(new com.arcrobotics.ftclib.geometry.Pose2d());
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        configureOtos();
    }
    public Pose2d getPoseEstimateOpticalRegular() {
        return new Pose2d(myOtos.getPosition().x, myOtos.getPosition().y, myOtos.getPosition().h);
    }
    public SparkFunOTOS.Pose2D getPoseEstimateOptical(){
        return myOtos.getPosition();
    }
    public void setPoseEstimateOptical(com.arcrobotics.ftclib.geometry.Pose2d newPose){
        SparkFunOTOS.Pose2D fancyPose = new SparkFunOTOS.Pose2D(newPose.getX(), newPose.getY(), newPose.getHeading());
        myOtos.setPosition(fancyPose);
    }

    public void test(SparkFunOTOS.Pose2D newPose) {
        myOtos.setPosition(newPose);
    }

    private void configureOtos() {
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.RADIANS);

        myOtos.calibrateImu();
        myOtos.resetTracking();

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(4.09705, 3.1539, Math.toRadians(-90)); //x = -8.1941 y = -6.3078
        myOtos.setOffset(offset);

        myOtos.setLinearScalar(1.00908);
        myOtos.setAngularScalar(0.99319);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);
    }

    public void setOtosPosition(double x, double y, double h) {
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(x, y, h);
        myOtos.setPosition(currentPosition);
    }

    @Override
    public void updatePose(com.arcrobotics.ftclib.geometry.Pose2d newPose) {

    }

    @Override
    public void updatePose() {

    }
}
