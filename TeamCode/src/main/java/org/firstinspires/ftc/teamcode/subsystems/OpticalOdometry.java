package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OpticalOdometry {

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
}
