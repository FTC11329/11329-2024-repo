package org.firstinspires.ftc.teamcode.utility;


import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.TimeUnit;


public class AprilTagDetectionPipeline {
    private static final boolean USE_WEBCAM = true;

    private ElapsedTime runtime = new ElapsedTime();

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    Telemetry telemetry;

    public void AprilTagInit(WebcamName webcam, Telemetry telemetry) {
        this.telemetry = telemetry;
        initAprilTag(webcam);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
//        setManualExposure(6, 250);
    }

    public void AprilTagStop() {
        visionPortal.close();
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag(WebcamName webcam) {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera
        builder.setCamera(webcam);

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
//        builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Function to add telemetry about AprilTag detections.
     */
    @SuppressLint("DefaultLocale")
    public void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        // Add legend to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }
    //im not ready to delete this yet but it might be coming
    /*
    double straightSpeedMultiplier = 0.02;
    double strafeSpeedMultiplier  = 0.02;
    double turnSpeedMultiplier  = -0.006;

    public List<Double> moveToBackdrop() {
        // initializing variables;

        boolean seen1;
        boolean seen2;
        boolean seen3;

        AprilTagDetection AprilTag1 = null;
        AprilTagDetection AprilTag2 = null;
        AprilTagDetection AprilTag3 = null;

        List<Double> driveList1 = new ArrayList<Double>();
        List<Double> driveList2 = new ArrayList<Double>();
        List<Double> driveList3 = new ArrayList<Double>();
        List<Double> driveListFinal = new ArrayList<Double>();

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();


        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 1) {
                AprilTag1 = detection;
            } else if (detection.id == 2) {
                AprilTag2 = detection;
            } else if (detection.id == 3) {
                AprilTag3 = detection;
            }
        }

        //makes the first april tag list (or dies trying)
        if (AprilTag1 != null) {
            double forwardSpeed1;
            double turnSpeed1;
            double strafeSpeed1;

            //Calculating all the speed and stuff
            forwardSpeed1 = (AprilTag1.ftcPose.y - 150) * straightSpeedMultiplier;
            strafeSpeed1  = (AprilTag1.ftcPose.x + 81) * strafeSpeedMultiplier;

            turnSpeed1 = AprilTag1.ftcPose.yaw * turnSpeedMultiplier;

            driveList1.add(0, forwardSpeed1);
            driveList1.add(1, strafeSpeed1);
            driveList1.add(2, turnSpeed1);
            seen1 = true;
        } else {
            seen1 = false;
        }

        //makes the second april tag ---------------------------------------------------------------
        if (AprilTag2 != null) {
            double forwardSpeed2;
            double turnSpeed2;
            double strafeSpeed2;

            //Calculating all the speed and stuff
            forwardSpeed2 = (AprilTag2.ftcPose.y - 138.3) * straightSpeedMultiplier;
            strafeSpeed2  = (AprilTag2.ftcPose.x + 16.5) * strafeSpeedMultiplier;

            turnSpeed2 = AprilTag2.ftcPose.yaw * turnSpeedMultiplier;

            driveList2.add(0, forwardSpeed2);
            driveList2.add(1, strafeSpeed2);
            driveList2.add(2, turnSpeed2);
            seen2 = true;
        } else {
            seen2 = false;
        }

        //makes the third april tag ----------------------------------------------------------------
        if (AprilTag3 != null) {
            double forwardSpeed3;
            double turnSpeed3;
            double strafeSpeed3;

            //Calculating all the speed and stuff
            forwardSpeed3 = (AprilTag3.ftcPose.y - 138.3) * straightSpeedMultiplier;
            strafeSpeed3  = (AprilTag3.ftcPose.x - 43.3) * strafeSpeedMultiplier;

            turnSpeed3 = AprilTag3.ftcPose.yaw * turnSpeedMultiplier;

            driveList3.add(0, forwardSpeed3);
            driveList3.add(1, strafeSpeed3);
            driveList3.add(2, turnSpeed3);
            seen3 = true;
        } else {
            seen3 = false;
        }

        //takes the mean of both lists if both are seen
        if (seen1 && !seen2 && !seen3){
            //only sees 1
            driveListFinal.add(0, driveList1.get(0));
            driveListFinal.add(1, driveList1.get(1));
            driveListFinal.add(2, driveList1.get(2));
            //outputs 1 if we see anything for auto
            driveListFinal.add(3, 1.0);
        } else if (!seen1 && seen2 && !seen3) {
            //only sees 2
            driveListFinal.add(0, driveList2.get(0));
            driveListFinal.add(1, driveList2.get(1));
            driveListFinal.add(2, driveList2.get(2));
            //outputs 1 if we see anything for auto
            driveListFinal.add(3, 1.0);
        } else if (!seen1 && !seen2 && seen3) {
            //only sees 3
            driveListFinal.add(0, driveList3.get(0));
            driveListFinal.add(1, driveList3.get(1));
            driveListFinal.add(2, driveList3.get(2));
            //outputs 1 if we see anything for auto
            driveListFinal.add(3, 1.0);
        } else if (seen1 && seen2 && !seen3) {
            //sees 1 and 2
            driveListFinal.add(0, (driveList1.get(0) + driveList2.get(0)) / 2);
            driveListFinal.add(1, (driveList1.get(1) + driveList2.get(1)) / 2);
            driveListFinal.add(2, (driveList1.get(2) + driveList2.get(2)) / 2);
            //outputs 1 if we see anything for auto
            driveListFinal.add(3, 1.0);
        } else if (!seen1 && seen2 && seen3) {
            //sees 2 and 3
            driveListFinal.add(0, (driveList2.get(0) + driveList3.get(0)) / 2);
            driveListFinal.add(1, (driveList2.get(1) + driveList3.get(1)) / 2);
            driveListFinal.add(2, (driveList2.get(2) + driveList3.get(2)) / 2);
            //outputs 1 if we see anything for auto
            driveListFinal.add(3, 1.0);
        } else if (seen1 && !seen2 && seen3) {
            //sees 1 and 3
            driveListFinal.add(0, (driveList1.get(0) + driveList3.get(0)) / 2);
            driveListFinal.add(1, (driveList1.get(1) + driveList3.get(1)) / 2);
            driveListFinal.add(2, (driveList1.get(2) + driveList3.get(2)) / 2);
            //outputs 1 if we see anything for auto
            driveListFinal.add(3, 1.0);
        } else if (seen1 && seen2 && seen3) {
            //sees everything
            driveListFinal.add(0, (driveList1.get(0) + driveList2.get(0) + driveList3.get(0)) / 3);
            driveListFinal.add(1, (driveList1.get(1) + driveList2.get(1) + driveList3.get(1)) / 3);
            driveListFinal.add(2, (driveList1.get(2) + driveList2.get(2) + driveList3.get(2)) / 3);
            //outputs 1 if we see anything for auto
            driveListFinal.add(3, 1.0);
        } else {
            //sees nothing
            driveListFinal.add(0, 0.0);
            driveListFinal.add(1, 0.0);
            driveListFinal.add(2, 0.0);
            //outputs 0 if we see nothing for auto
            driveListFinal.add(3, 0.0);
        }

        //if is in tolerance
        if (Math.abs(driveListFinal.get(0) + driveListFinal.get(1) + driveListFinal.get(2)) < 2) {
//            driveListFinal.add(0, 0.0);
//            driveListFinal.add(1, 0.0);
//            driveListFinal.add(2, 0.0);
            //outputs 3 if we are in position for auto
//            driveListFinal.add(3, 3.0);
        }
        return driveListFinal;
    }


    //Truss April tag

    public List<Double> moveToTruss() {
        // initializing variables;
        boolean seen1;
        boolean seen2;

        AprilTagDetection AprilTag9 = null;
        AprilTagDetection AprilTag10 = null;

        List<Double> driveList1 = new ArrayList<Double>();
        List<Double> driveList2 = new ArrayList<Double>();
        List<Double> driveListFinal = new ArrayList<Double>();

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();


        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 9) {
                AprilTag9 = detection;
            } else if (detection.id == 10) {
                AprilTag10 = detection;
            }
        }
        //makes the first april tag list
        if (AprilTag9 != null) {
            double forwardSpeed1;
            double turnSpeed1;
            double strafeSpeed1;

            //Calculating all the speed and stuff
            forwardSpeed1 = (AprilTag9.ftcPose.y - 130.7) * straightSpeedMultiplier;
            strafeSpeed1 = (AprilTag9.ftcPose.x + 89.4) * strafeSpeedMultiplier;

            turnSpeed1 = AprilTag9.ftcPose.yaw * turnSpeedMultiplier;

            driveList1.add(0, forwardSpeed1);
            driveList1.add(1, strafeSpeed1);
            driveList1.add(2, turnSpeed1);
            seen1 = true;
        } else {
            seen1 = false;
        }

        //makes the second april tag list
        if (AprilTag10 != null) {
            double forwardSpeed2;
            double turnSpeed2;
            double strafeSpeed2;

            //Calculating all the speed and stuff
            forwardSpeed2 = (AprilTag10.ftcPose.y - 127.2) * straightSpeedMultiplier;
            strafeSpeed2 = (AprilTag10.ftcPose.x + 33.9) * strafeSpeedMultiplier;

            turnSpeed2 = AprilTag10.ftcPose.yaw * turnSpeedMultiplier;

            driveList2.add(0, forwardSpeed2);
            driveList2.add(1, strafeSpeed2);
            driveList2.add(2, turnSpeed2);
            seen2 = true;
        } else {
            seen2 = false;
        }

        //calculating average
        if (seen1 && !seen2){
            //only sees 1
            driveListFinal.add(0, driveList1.get(0));
            driveListFinal.add(1, driveList1.get(1));
            driveListFinal.add(2, driveList1.get(2));
            //outputs 1 if we see anything for auto
            driveListFinal.add(3, 1.0);
        } else if (!seen1 && seen2) {
            //only sees 2
            driveListFinal.add(0, driveList2.get(0));
            driveListFinal.add(1, driveList2.get(1));
            driveListFinal.add(2, driveList2.get(2));
            //outputs 1 if we see anything for auto
            driveListFinal.add(3, 1.0);
        } else if (seen1 && seen2) {
            //sees 1 and 2
            driveListFinal.add(0, (driveList1.get(0) + driveList2.get(0)) / 2);
            driveListFinal.add(1, (driveList1.get(1) + driveList2.get(1)) / 2);
            driveListFinal.add(2, (driveList1.get(2) + driveList2.get(2)) / 2);
            //outputs 1 if we see anything for auto
            driveListFinal.add(3, 1.0);
        } else {
            //sees nothing
            driveListFinal.add(0,0.0);
            driveListFinal.add(1,0.0);
            driveListFinal.add(2,0.0);
            //outputs 0 if we see nothing for auto
            driveListFinal.add(3, 0.0);
        }
        //in tolerance and sees something
        if (Math.abs(driveListFinal.get(0) + driveListFinal.get(1)) < 0.1 && driveListFinal.get(3) == 1.0) {
            driveListFinal.add(0, 0.0);
            driveListFinal.add(1, 0.0);
            driveListFinal.add(2, 0.0);
            //outputs 3 if we are in position for auto
            driveListFinal.add(3, 3.0);
        }

        return driveListFinal;
    }
 */

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    boolean targetFound = false;    // Set to true when an AprilTag target is detected
    double drive = 0;        // Desired forward power/speed (-1 to +1)
    double strafe = 0;        // Desired strafe power/speed (-1 to +1)
    double turn = 0;        // Desired turning power/speed (-1 to +1)

    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    List<Double> driveList = new ArrayList<>();

    public List<Double> autoAprilTag(int DESIRED_TAG_ID, double DESIRED_DISTANCE, double DESIRED_LATERAL_DISTANCE) {
        targetFound = false;
        desiredTag = null;
        driveList = new ArrayList<>();;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically
        if (desiredTag != null) {
            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double yawError = desiredTag.ftcPose.yaw - DESIRED_LATERAL_DISTANCE;
            double headingError = desiredTag.ftcPose.bearing;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            driveList.add(0, drive);
            driveList.add(1, strafe);
            driveList.add(2, turn);

            if (targetFound) {
                driveList.add(3, 1.0);
            } else {
                driveList.add(3, 0.0);
            }
//            if (rangeError + headingError - yawError < 1 && driveList.get(3) == 1.0) {
//                driveList.clear();
//                driveList.add(0, 0.0);
//                driveList.add(1, 0.0);
//                driveList.add(2, 0.0);
//                driveList.add(3, 3.0);
//            }
        } else {
            driveList.add(0, 0.0);
            driveList.add(1, 0.0);
            driveList.add(2, 0.0);
            driveList.add(3, 0.0);
        }
        return driveList;
    }

    public Optional<AprilTagDetection> findAprilTag(int id) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == id && detection.metadata != null) {
                return Optional.of(detection);
            }
        }
        return Optional.empty();
    }

    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls
        double startTime = runtime.milliseconds();
        if (visionPortal == null) {
            return;
        }
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                while (runtime.milliseconds() - startTime < 20) ;
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
        // Make sure camera is streaming before we try to set the exposure controls
        // Set camera controls unless we are stopping.
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            while (runtime.milliseconds() - startTime > 50) ;
        }
        exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
        while (runtime.milliseconds() - startTime > 20) ;
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        while (runtime.milliseconds() - startTime > 20) ;
    }
}
