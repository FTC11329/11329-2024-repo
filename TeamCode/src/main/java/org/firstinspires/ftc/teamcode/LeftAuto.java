package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

@Autonomous(name = "Left Auto", group = "Competition")
public class LeftAuto extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    Intake intake;
    WebcamName webcam1;
    Drivetrain drivetrain;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    @Override
    public void init(){
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline();
        telemetry.addData("Status", "Initialized");
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        aprilTagDetectionPipeline.AprilTagInit(webcam1, telemetry);
    }

    @Override
    public void start() {
        //in starting position, rotating right until it sees a tag
        while (aprilTagDetectionPipeline.moveToTruss().get(3) == 0) {
            drivetrain.drive(0, 0, 0.25, gamepad1.a);
        }

        //moving to truss
        while (aprilTagDetectionPipeline.moveToTruss().get(3) != 3) {
            List<Double> driveList = aprilTagDetectionPipeline.moveToTruss();
            drivetrain.drive(driveList.get(0), driveList.get(1), driveList.get(2), gamepad1.a);
        }
        //turning fast
        while (aprilTagDetectionPipeline.moveToTruss().get(3) == 3) {
            drivetrain.drive(0, 0, -0.5, gamepad1.a);
        }

        //turning slow
        while (aprilTagDetectionPipeline.moveToBackdrop().get(3) == 0) {
            drivetrain.drive(0,0,-0.15, gamepad1.a);
        }

        while (aprilTagDetectionPipeline.moveToTruss().get(3) != 3) {
            List<Double> driveList = aprilTagDetectionPipeline.moveToBackdrop();
            drivetrain.drive(driveList.get(0), driveList.get(1), driveList.get(2), gamepad1.a);
        }

    }

    @Override
    public void loop() {
        //literally nothing
    }

    @Override
    public void stop() {
        drivetrain.drive(0,0,0, true);
    }

}
