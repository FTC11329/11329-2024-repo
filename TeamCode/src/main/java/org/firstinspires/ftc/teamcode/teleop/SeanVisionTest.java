package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.BarcodePipeline;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Sean Vision Test", group = "test")
public class SeanVisionTest extends OpMode {
    WebcamName webcamName;
    BarcodePipeline barcode = new BarcodePipeline();
    OpenCvWebcam webcam;

    public Integer cameraMonitorViewId;
    boolean webcamReady = false;

    @Override
    public void init() {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam.setPipeline(barcode);

        webcam.setMillisecondsPermissionTimeout(4000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

                webcamReady = true;
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error :(", errorCode);
            }
        });

    }

    @Override
    public void loop() {
            telemetry.addData("Barcode ID", barcode.position);
            // Pose2d finalUpdatePose = new Pose2d(0, 0, 0);
            // drivetrain.setPoseEstimate(finalUpdatePose)
    }
}
