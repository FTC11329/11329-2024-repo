package org.firstinspires.ftc.teamcode.helpers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.vision.processors.DashboardCameraStreamProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class ReadWebcam extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final DashboardCameraStreamProcessor processor = new DashboardCameraStreamProcessor();

        WebcamName webcam = hardwareMap.get(WebcamName.class, Constants.Vision.frontWebcamName);

        new VisionPortal.Builder().addProcessor(processor).setCamera(webcam).build();

        FtcDashboard.getInstance().startCameraStream(processor, 0);

        waitForStart();

        while (opModeIsActive()) {
            sleep(100L);
        }
    }
}