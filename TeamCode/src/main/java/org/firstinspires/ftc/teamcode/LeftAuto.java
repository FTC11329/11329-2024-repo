package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
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

        boolean oneTimeVariable = true;

        waitForStart();

        int stage = -1;

        while (opModeIsActive()) {
            List<Double> backDropTag = new ArrayList<>();
            List<Double> stackTag = new ArrayList<>();


            aprilTagDetectionPipeline.telemetryAprilTag();
            telemetry.addData("stage = ", stage);
            telemetry.addData("Drive list", stackTag);
            telemetry.update();
            boolean go = gamepad1.a;
            //going forward
            if (stage == -1) {
                double startTime = 0.0;
                if (oneTimeVariable) {
                    startTime = runtime.seconds();
                    oneTimeVariable = false;
                }

                drivetrain.drive(0.5, 0,0, true);

                if (runtime.seconds() - startTime > 5) {
                    drivetrain.drive(0, 0,0, true);
                    stage = 0;
                    oneTimeVariable = true;
                }
            }
            //in starting position, rotating right until it sees a tag
            if (stackTag.get(3) == 0 && stage == 0) {
                drivetrain.drive(0, 0, 0.15, go);
                if (stackTag.get(3) == 1) {
                    stage = 1;
                }
            }

            //moving to truss
            if (stackTag.get(3) != 3 && stage == 1) {
                List<Double> driveList = stackTag;
                drivetrain.drive(driveList.get(0), driveList.get(1), driveList.get(2), go);
                if (stackTag.get(3) == 3) {
                    stage = 2;
                }
            }

            //moving under truss
            if (stackTag.get(3) == 3 && stage == 2) {
                List<Double> driveList = stackTag;
                drivetrain.drive(-0.5, 0, driveList.get(2) * 1.5, go);
                if (stackTag.get(3) != 3) {
                    stage = 3;
                }
            }

            //turning around slowly
            if (backDropTag.get(3) == 0 && stage == 3) {
                drivetrain.drive(0, 0, -0.15, go);
                if (backDropTag.get(3) == 1) {
                    stage = 4;
                }
            }
            //moving to back drop
            if (backDropTag.get(3) != 3 && stage == 4) {
                List<Double> driveList = backDropTag;
                drivetrain.drive(driveList.get(0), driveList.get(1), driveList.get(2), go);
                if (backDropTag.get(3) == 3) {
                    stage = 5;
                }
            }
            //outtake pixel
            if (stage == 5) {
                double startTime = 0.0;
                if (oneTimeVariable) {
                    startTime = runtime.seconds();
                    oneTimeVariable = false;
                }

                intake.setIntakePower(-0.25);

                if (runtime.seconds() - startTime > 1) {
                    intake.setIntakePower(0);
                    stage = 6;
                }
            }
        }
    }
}



/*
            drivetrain.drive(0.8,0,0,go);
            //in starting position, rotating right until it sees a tag
            if (stackTag.get(3) == 0 && stage == 0) {
                drivetrain.drive(0, 0, 0.25, go);
                /*
                if (stackTag.get(3) == 1) {
                    stage = 1;
                }

            }

                    //moving to truss
                    if (stackTag.get(3) != 3 && stage == 1) {
                    List<Double> driveList = stackTag;
        drivetrain.drive(driveList.get(0), driveList.get(1), driveList.get(2), go);
        if (stackTag.get(3) == 3) {
        stage = 2;
        }
        }

        //turning fast
        if (stackTag.get(3) == 3 && stage == 2) {
        drivetrain.drive(0, 0, -0.5, go);
        if (stackTag.get(3) != 3) {
        stage = 3;
        }
        }

        //turning slow
        if (backDropTag.get(3) == 0 && stage == 3) {
        drivetrain.drive(0, 0, -0.15, go);
        if (backDropTag.get(3) != 0) {
        stage = 4;
        }
        }

        if (stackTag.get(3) != 3 && stage == 4) {
        List<Double> driveList = backDropTag;
        drivetrain.drive(driveList.get(0), driveList.get(1), driveList.get(2), go);
        if (backDropTag.get(3) == 3) {
        stage = 5;
        }
        }

 */
