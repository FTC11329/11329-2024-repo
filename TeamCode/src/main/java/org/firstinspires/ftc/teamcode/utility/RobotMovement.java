package org.firstinspires.ftc.teamcode.utility;

import static java.lang.Math.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.OpticalOdometry;
import org.opencv.core.Point;

import java.util.ArrayList;

public class RobotMovement {

    Pose2d movementPose;

    OpticalOdometry opticalOdometry;

    public RobotMovement(HardwareMap hardwareMap) {
        opticalOdometry = new OpticalOdometry(hardwareMap);
    }

    public Pose2d goToPosition(Pose2d newPose, double movementSpeed, double turnSpeed, double positionOffset) {
        return goToPosition(newPose.getX(), newPose.getY(), newPose.getHeading(), movementSpeed, turnSpeed, positionOffset);
    }

    public Pose2d goToPosition(double x, double y, double preferredAngle, double movementSpeed, double turnSpeed, double positionOffset) {

        double distanceToTarget = Math.hypot(x - opticalOdometry.getX(), y - opticalOdometry.getY());

        double absoluteAngleToTarget = Math.atan2(y - opticalOdometry.getY(), x - opticalOdometry.getX());

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (opticalOdometry.getHeading() - Math.toRadians(90)));


        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint));


        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;

        double movementTurnPower = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;

        if (distanceToTarget < positionOffset) {
            movementTurnPower = 0;
        }

        movementPose = new Pose2d(movementXPower * movementSpeed, movementYPower * movementSpeed, movementTurnPower);
        return movementPose;
    }





    //MATH FUNCTIONS********************************************************************************


    public static double AngleWrap (double angle){
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }


    public ArrayList<Point> lineCircleIntersection (Point circleCenter, double radius, Point linePoint1, Point linePoint2) {


        if (Math.abs(linePoint1.y - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 0.003;
        }
        if (Math.abs(linePoint1.x - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + 0.003;
        }


        double m1 = (linePoint2.y - linePoint1.y)/(linePoint2.x - linePoint2.y);

        double quadraticA = 1.0 + pow(m1, 2);

        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;


        double quadraticB = (2.0 * m1 * y1) - (2.0 * pow(m1,2) * x1);
        double quadraticC = ((pow(m1, 2) * pow(x1,2))) - (2.0*y1*m1*x1) + pow(y1,2) - pow(radius,2);
        ArrayList<Point> allPoints = new ArrayList<>();
        try {

            double xRoot1 = (-quadraticB + sqrt(pow(quadraticB, 2)-(4.0 * quadraticA * quadraticC)))/(2.0*quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;


            double minX = linePoint1.x < linePoint1.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint1.x ? linePoint1.x : linePoint2.x;

            if (xRoot1 > minX && xRoot1 < maxX) {
                allPoints.add(new Point(xRoot1, yRoot1));
            }

            double xRoot2 = (-quadraticB - sqrt(pow(quadraticB, 2)-(4.0 * quadraticA * quadraticC)))/(2.0*quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            if (xRoot1 > minX && xRoot1 < maxX) {
                allPoints.add(new Point(xRoot2, yRoot2));
            }

        } catch (Exception e) {

        }
        return allPoints;
    }
}
