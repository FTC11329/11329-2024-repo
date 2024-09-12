package org.firstinspires.ftc.teamcode.vision;


import android.annotation.SuppressLint;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utility.RunAfterTime;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.TimeUnit;


public class AprilTagDetectionPipeline {
    private final ElapsedTime runtime = new ElapsedTime();

    /**
     * Initialize the AprilTag processor.
     */
    public static AprilTagProcessor createAprilTagProcessor() {
        AprilTagLibrary myLibrary = new AprilTagLibrary.Builder()
                .setAllowOverwrite(true)
                .addTags(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .addTag(11, "SharedAllianceLeft",
                        2, new VectorF(60.25f, 41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(12, "SharedAllianceCenter",
                        2, new VectorF(60.25f, 41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(13, "SharedAllianceRight",
                        2, new VectorF(60.25f, 41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .build();
        // Create the AprilTag processor.
        return new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(myLibrary)
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //for Webcam
//                .setLensIntrinsics(934.078, 934.078, 332.145, 257.176)
                //for global shudder
                .setLensIntrinsics(903.79, 903.79, 699.758, 372.872)
                // ... these parameters are fx, fy, cx, cy.
                .build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);
    }

    //Idk what this function actually does but it makes the robot run smooth so ya it stays.
    private static void setManualExposure(VisionPortal visionPortal, VisionPortal visionPortal2, int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null || visionPortal2 == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            RunAfterTime.runAfterTime(10, () -> {
            });
        }

        // Set camera controls
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            RunAfterTime.runAfterTime(50, () -> {
            });
        }

        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
        RunAfterTime.runAfterTime(20, () -> {
        });

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        RunAfterTime.runAfterTime(20, () -> {
        });
    }

    @SuppressLint("DefaultLocale")
    public void telemetryAprilTag(Telemetry telemetry, List<AprilTagDetection> currentDetections) {

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

    public static Optional<AprilTagDetection> getDesiredTag(List<AprilTagDetection> currentDetections, int id) {
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == id && detection.metadata != null) {
                return Optional.of(detection);
            }
        }
        return Optional.empty();
    }
}
