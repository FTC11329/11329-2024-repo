package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

//@TeleOp(name = "Left Auto", group = "Competition")
@Autonomous(name = "Left Auto", group = "Competition")
public class LeftAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    Intake intake;
    WebcamName webcam1;
    Drivetrain drivetrain;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    @Override
    public void runOpMode() {
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline();
        telemetry.addData("Status", "Initialized");
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        aprilTagDetectionPipeline.AprilTagInit(webcam1, telemetry);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();
        drivetrain.driveSpeed = DriveSpeedEnum.Auto;

        waitForStart();

        int stage = 0;

        while (opModeIsActive()) {
            aprilTagDetectionPipeline.telemetryAprilTag();
            telemetry.addData("stage = ", stage);
            telemetry.addData("Drive list", aprilTagDetectionPipeline.moveToTruss());
            telemetry.update();

            //in starting position, rotating right until it sees a tag
            if (aprilTagDetectionPipeline.moveToTruss().get(3) == 0 && stage == 0) {
                drivetrain.drive(0, 0, 0.25, gamepad1.a);
                if (aprilTagDetectionPipeline.moveToTruss().get(3) == 1) {
                    stage = 1;
                }
            }

            //moving to truss
            if (aprilTagDetectionPipeline.moveToTruss().get(3) != 3 && stage == 1) {
                List<Double> driveList = aprilTagDetectionPipeline.moveToTruss();
                drivetrain.drive(driveList.get(0), driveList.get(1), driveList.get(2), gamepad1.a);
                if (aprilTagDetectionPipeline.moveToTruss().get(3) == 3) {
                    stage = 2;
                }
            }

            //turning fast
            if (aprilTagDetectionPipeline.moveToTruss().get(3) == 3) {
                drivetrain.drive(0, 0, -0.5, gamepad1.a);
            }
/*
            //turning slow
            if (aprilTagDetectionPipeline.moveToBackdrop().get(3) == 0) {
                drivetrain.drive(0, 0, -0.15, gamepad1.a);
            }

            if (aprilTagDetectionPipeline.moveToTruss().get(3) != 3) {
                List<Double> driveList = aprilTagDetectionPipeline.moveToBackdrop();
                drivetrain.drive(driveList.get(0), driveList.get(1), driveList.get(2), gamepad1.a);
            }

 */
        }
    }
}



/*
            drivetrain.drive(0.8,0,0,gamepad1.a);
            //in starting position, rotating right until it sees a tag
            if (aprilTagDetectionPipeline.moveToTruss().get(3) == 0 && stage == 0) {
                drivetrain.drive(0, 0, 0.25, gamepad1.a);
                /*
                if (aprilTagDetectionPipeline.moveToTruss().get(3) == 1) {
                    stage = 1;
                }

            }

                    //moving to truss
                    if (aprilTagDetectionPipeline.moveToTruss().get(3) != 3 && stage == 1) {
                    List<Double> driveList = aprilTagDetectionPipeline.moveToTruss();
        drivetrain.drive(driveList.get(0), driveList.get(1), driveList.get(2), gamepad1.a);
        if (aprilTagDetectionPipeline.moveToTruss().get(3) == 3) {
        stage = 2;
        }
        }

        //turning fast
        if (aprilTagDetectionPipeline.moveToTruss().get(3) == 3 && stage == 2) {
        drivetrain.drive(0, 0, -0.5, gamepad1.a);
        if (aprilTagDetectionPipeline.moveToTruss().get(3) != 3) {
        stage = 3;
        }
        }

        //turning slow
        if (aprilTagDetectionPipeline.moveToBackdrop().get(3) == 0 && stage == 3) {
        drivetrain.drive(0, 0, -0.15, gamepad1.a);
        if (aprilTagDetectionPipeline.moveToBackdrop().get(3) != 0) {
        stage = 4;
        }
        }

        if (aprilTagDetectionPipeline.moveToTruss().get(3) != 3 && stage == 4) {
        List<Double> driveList = aprilTagDetectionPipeline.moveToBackdrop();
        drivetrain.drive(driveList.get(0), driveList.get(1), driveList.get(2), gamepad1.a);
        if (aprilTagDetectionPipeline.moveToBackdrop().get(3) == 3) {
        stage = 5;
        }
        }

 */
