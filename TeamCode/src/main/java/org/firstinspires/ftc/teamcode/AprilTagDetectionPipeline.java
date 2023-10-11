package org.firstinspires.ftc.teamcode;


import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.lang.Math;



public class AprilTagDetectionPipeline {
    private static final boolean USE_WEBCAM = true;
    /**
     * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    Telemetry telemetry;

    public void AprilTagInit(WebcamName webcam, Telemetry telemetry) {
        this.telemetry = telemetry;
        initAprilTag(webcam);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
    }

        public void AprilTagOnTick() {
        // Push telemetry to the Driver Station.
        moveToAprilTag();
        }

        public void AprilTagStop() {
            visionPortal.close();
        }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag(WebcamName webcam) {
        AprilTagLibrary.Builder myAprilTagLibraryBuilder;
        AprilTagLibrary myAprilTagLibrary;

        myAprilTagLibraryBuilder = new AprilTagLibrary.Builder()
                .addTag(1, "WOOF", 0.5D, new VectorF(0.0F, 0.0F, 0.0F), DistanceUnit.METER, Quaternion.identityQuaternion())
                .addTag(2, "OINK", 0.5D, new VectorF(0.0F, 0.0F, 0.0F), DistanceUnit.METER, Quaternion.identityQuaternion())
                .addTag(3, "MOO",  0.5D, new VectorF(0.0F, 0.0F, 0.0F), DistanceUnit.METER, Quaternion.identityQuaternion());

        myAprilTagLibrary = myAprilTagLibraryBuilder.build();
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(myAprilTagLibrary)
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
        builder.enableCameraMonitoring(true);

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

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }
    double normalizedTurnSpeed1;
    double normalizedForwardSpeed1;
    double normalizedStrafeSpeed1;

    double normalizedTurnSpeed2;
    double normalizedForwardSpeed2;
    double normalizedStrafeSpeed2;

    double normalizedTurnSpeed3;
    double normalizedForwardSpeed3;
    double normalizedStrafeSpeed3;

    public List<Double> moveToAprilTag() {
        // initializing variables;
        double straightSpeedMultiplier = 0.02;
        double strafeSpeedMultiplier  = 0.02;
        double turnSpeedMultiplier  = -0.02;

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
        //makes the first april tag list
        if (AprilTag1 != null) {
            double forwardSpeed1;
            double turnSpeed1;
            double strafeSpeed1;

            //Calculating all the speed and stuff
            forwardSpeed1 = (AprilTag1.ftcPose.y - 150) * straightSpeedMultiplier;
            strafeSpeed1  = (AprilTag1.ftcPose.x + 10.16) * strafeSpeedMultiplier;
//            forwardSpeed = 0.05 * (AprilTag2.ftcPose.y - 30);
//            strafeSpeed = 0.05 * (AprilTag2.ftcPose.z - 10);

            double yaw = AprilTag1.ftcPose.yaw;
            turnSpeed1 = (yaw) * turnSpeedMultiplier;
            //Exclude 180 from the domain
//            if (yaw == 180) {
//                yaw += 1;
//            }
            //Really cool math that would have worked for turning
//            turnSpeed = (Math.sin((Math.PI / 180) * yaw) * (Math.abs(yaw - 180)) / (yaw - 180));

            //Takes the max of all the speeds and sets that to 1
            //Then takes the ratio of the other values with the max and now they are all reasonable values
            double max = Math.max(Math.max(1, turnSpeed1), Math.max(forwardSpeed1, strafeSpeed1));
            normalizedForwardSpeed1 = forwardSpeed1 / max;
            normalizedStrafeSpeed1 = strafeSpeed1 / max;
            normalizedTurnSpeed1  = turnSpeed1  / max;

            driveList1.add(0, normalizedForwardSpeed1);
            driveList1.add(1, normalizedStrafeSpeed1);
            driveList1.add(2, normalizedTurnSpeed1);
            seen1 = true;
        } else {
            // if nothing was seen then we use the last known values
            driveList1.add(0, normalizedForwardSpeed1);
            driveList1.add(1, normalizedStrafeSpeed1);
            driveList1.add(2, normalizedTurnSpeed1);
            seen1 = false;
        }


        //makes the second april tag ---------------------------------------------------------------
        if (AprilTag2 != null) {
            double forwardSpeed2;
            double turnSpeed2;
            double strafeSpeed2;

            //Calculating all the speed and stuff
            forwardSpeed2 = (AprilTag2.ftcPose.y - 150) * straightSpeedMultiplier;
            strafeSpeed2  = (AprilTag2.ftcPose.x) * strafeSpeedMultiplier;
//            forwardSpeed = 0.05 * (AprilTag2.ftcPose.y - 30);
//            strafeSpeed = 0.05 * (AprilTag2.ftcPose.z - 10);

            double yaw = AprilTag2.ftcPose.yaw;
            turnSpeed2 = (yaw) * turnSpeedMultiplier;

            //Exclude 180 from the domain
//            if (yaw == 180) {
//                yaw += 1;
//            }
            //Really cool math for turning
//            turnSpeed = (Math.sin((Math.PI / 180) * yaw) * (Math.abs(yaw - 180)) / (yaw - 180));

            //Takes the max of all the speeds and sets that to 1
            //Then takes the ratio of the other values with the max and now they are all reasonable values
            double max = Math.max(Math.max(1, turnSpeed2), Math.max(forwardSpeed2, strafeSpeed2));
            normalizedForwardSpeed2 = forwardSpeed2 / max;
            normalizedStrafeSpeed2 = strafeSpeed2 / max;
            normalizedTurnSpeed2  = turnSpeed2  / max;

            driveList2.add(0, normalizedForwardSpeed2);
            driveList2.add(1, normalizedStrafeSpeed2);
            driveList2.add(2, normalizedTurnSpeed2);
            seen2 = true;
        } else {
            // if nothing was seen then we use the last known values
            driveList2.add(0, normalizedForwardSpeed2);
            driveList2.add(1, normalizedStrafeSpeed2);
            driveList2.add(2, normalizedTurnSpeed2);
            seen2 = false;
        }


        //makes the third april tag ----------------------------------------------------------------
        if (AprilTag3 != null) {
            double forwardSpeed3;
            double turnSpeed3;
            double strafeSpeed3;

            //Calculating all the speed and stuff
            forwardSpeed3 = (AprilTag3.ftcPose.y - 150) * straightSpeedMultiplier;
            strafeSpeed3  = (AprilTag3.ftcPose.x - 10.16) * strafeSpeedMultiplier;
//            forwardSpeed = 0.05 * (AprilTag3.ftcPose.y - 30);
//            strafeSpeed = 0.05 * (AprilTag3.ftcPose.z - 10);

            double yaw = AprilTag3.ftcPose.yaw;
            turnSpeed3 = (yaw) * turnSpeedMultiplier;

            //Exclude 180 from the domain
//            if (yaw == 180) {
//                yaw += 1;
//            }
            //Really cool math for turning
//            turnSpeed = (Math.sin((Math.PI / 180) * yaw) * (Math.abs(yaw - 180)) / (yaw - 180));

            //Takes the max of all the speeds and sets that to 1
            //Then takes the ratio of the other values with the max and now they are all reasonable values
            double max = Math.max(Math.max(1, turnSpeed3), Math.max(forwardSpeed3, strafeSpeed3));
            normalizedForwardSpeed3 = forwardSpeed3 / max;
            normalizedStrafeSpeed3 = strafeSpeed3 / max;
            normalizedTurnSpeed3  = turnSpeed3  / max;

            driveList3.add(0, normalizedForwardSpeed3);
            driveList3.add(1, normalizedStrafeSpeed3);
            driveList3.add(2, normalizedTurnSpeed3);
            seen3 = true;
        } else {
            // if nothing was seen then we use the last known values
            driveList3.add(0, normalizedForwardSpeed3);
            driveList3.add(1, normalizedStrafeSpeed3);
            driveList3.add(2, normalizedTurnSpeed3);
            seen3 = false;
        }

        //takes the mean of both lists if both are seen
        if (seen1 && !seen2 && !seen3){
            //only sees 1
            driveListFinal.add(0, driveList1.get(0));
            driveListFinal.add(1, driveList1.get(1));
            driveListFinal.add(2, driveList1.get(2));
        } else if (!seen1 && seen2 && !seen3) {
            //only sees 2
            driveListFinal.add(0, driveList2.get(0));
            driveListFinal.add(1, driveList2.get(1));
            driveListFinal.add(2, driveList2.get(2));
        } else if (!seen1 && !seen2 && seen3) {
            //only sees 3
            driveListFinal.add(0, driveList3.get(0));
            driveListFinal.add(1, driveList3.get(1));
            driveListFinal.add(2, driveList3.get(2));
        } else if (seen1 && seen2 && !seen3) {
            //sees 1 and 2
            driveListFinal.add(0, (driveList1.get(0) + driveList2.get(0)) / 2);
            driveListFinal.add(1, (driveList1.get(1) + driveList2.get(1)) / 2);
            driveListFinal.add(2, (driveList1.get(2) + driveList2.get(2)) / 2);
        } else if (!seen1 && seen2 && seen3) {
            //sees 2 and 3
            driveListFinal.add(0, (driveList2.get(0) + driveList3.get(0)) / 2);
            driveListFinal.add(1, (driveList2.get(1) + driveList3.get(1)) / 2);
            driveListFinal.add(2, (driveList2.get(2) + driveList3.get(2)) / 2);
        } else if (seen1 && !seen2 && seen3) {
            //sees 1 and 3
            driveListFinal.add(0, (driveList1.get(0) + driveList3.get(0)) / 2);
            driveListFinal.add(1, (driveList1.get(1) + driveList3.get(1)) / 2);
            driveListFinal.add(2, (driveList1.get(2) + driveList3.get(2)) / 2);
        } else if (seen1 && seen2 && seen3) {
            //sees everything
            driveListFinal.add(0, (driveList1.get(0) + driveList2.get(0) + driveList3.get(0)) / 3);
            driveListFinal.add(1, (driveList1.get(1) + driveList2.get(1) + driveList3.get(1)) / 3);
            driveListFinal.add(2, (driveList1.get(2) + driveList2.get(2) + driveList3.get(2)) / 3);
        } else if (!seen1 && !seen2 && !seen3) {
            //sees nothing
//            driveListFinal.add(0, (driveList1.get(0) + driveList2.get(0) + driveList3.get(0)) / -3);
//            driveListFinal.add(1, (driveList1.get(1) + driveList2.get(1) + driveList3.get(1)) / -3);
//            driveListFinal.add(2, (driveList1.get(2) + driveList2.get(2) + driveList3.get(2)) / -3);
            driveListFinal.add(0, 0.0);
            driveListFinal.add(1, 0.0);
            driveListFinal.add(2, 0.0);
        }
        return driveListFinal;
    }
}
