package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.utility.RunAfterTime.runAfterTime;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Plane;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.utility.AprilTagToRoadRunner;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetector;
import org.firstinspires.ftc.teamcode.vision.AprilTagIntoPower;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Optional;

@TeleOp(name = "Tele-op", group = "Allen op mode")
public class Teleop extends OpMode {
    WebcamName webcam1;

    Claw claw;
//    Plane plane;
    Slides slides;
    Intake intake;
    Climber climber;
    Outtake outtake;
    Drivetrain drivetrain;

    AprilTagDetector aprilTagDetector;
    AprilTagIntoPower aprilTagIntoPower;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    double slidePower = 0;
    double climberPower = 0;

    Pose2d runnerPose10 = new Pose2d(0,0,0);
    Pose2d runnerPose9 = new Pose2d(0,0,0);



    @Override
    public void init() {
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        telemetry.addData("Status", "Initialized");

         claw = new Claw(hardwareMap);
//         plane = new Plane(hardwareMap);
        slides = new Slides(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        climber = new Climber(hardwareMap);

        drivetrain = new Drivetrain(hardwareMap, telemetry);

        aprilTagDetector = new AprilTagDetector();
        aprilTagIntoPower = new AprilTagIntoPower();
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline();

//        aprilTagDetectionPipeline.AprilTagInit(webcam1, telemetry);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        boolean autoAlignToTag = gamepad1.a;

        slidePower = gamepad1.right_stick_y;

        if (autoAlignToTag && false) {
            /*
            //creates a variable of an april tag detection
            Optional<AprilTagDetection> optionalAprilTagDetection = aprilTagDetectionPipeline.getDesiredTag(10);

            if (optionalAprilTagDetection.isPresent()) {
                //creates a variable with the pose of the april tag in the form (distance(x and y), yaw, bearing)
                Pose2d distance = aprilTagDetector.distanceFromAprilTag(optionalAprilTagDetection.get());

                //creates a pose of what powers the motors should be set to in the form (forward, strafe, turn)
                Pose2d power = aprilTagIntoPower.toPower(distance);

                //sets that pose to the motors
                drivetrain.setWeightedDrivePower(power);
            } else {
                //if we don't see a tag, then set the drive powers to 0)
                drivetrain.drive(0,0,0, DriveSpeedEnum.Auto);
            }
        } else if (gamepad1.b && false) {
            //creates a variable of an april tag detection
            Optional<AprilTagDetection> optionalAprilTagDetection = aprilTagDetectionPipeline.getDesiredTag(10);

            if (optionalAprilTagDetection.isPresent()) {
                //creates a variable with the pose of the april tag in the form (x, y, and yaw)
                Pose2d distance = aprilTagDetector.distanceFromAprilTagExact(optionalAprilTagDetection.get());

                //creates a pose of what powers the motors should be set to in the form (forward, strafe, turn)
                Pose2d power = aprilTagIntoPower.toPowerExact(distance);

                //sets that pose to the motors
                drivetrain.setWeightedDrivePower(power);
            } else {
                //if we don't see a tag, then set the drive powers to 0)
                drivetrain.drive(0,0,0, DriveSpeedEnum.Auto);
            }
*/
        } else {
            DriveSpeedEnum driveSpeed;
            if (gamepad1.right_bumper) {
                driveSpeed = DriveSpeedEnum.Fast;
            } else {
                driveSpeed = DriveSpeedEnum.Slow;
            }
            //Testing wheels
            /*
                if (gamepad1.x){
                    drivetrain.rightFront.setPower(0.25);
                } else {
                    drivetrain.rightFront.setPower(0);
                }

                if (gamepad1.y){
                    drivetrain.leftFront.setPower(0.25);
                } else {
                    drivetrain.leftFront.setPower(0);
                }

                if (gamepad1.b){
                    drivetrain.rightRear.setPower(0.25);
                } else {
                    drivetrain.rightRear.setPower(0);
                }

                if (gamepad1.a){
                    drivetrain.leftRear.setPower(0.25);
                } else {
                    drivetrain.leftRear.setPower(0);
                }
*/

            //DRIVETRAIN
            drivetrain.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x, driveSpeed);
            //for avalanche

            //SLIDES
            slidePower = -gamepad1.right_stick_y;
            slides.manualPosition(slidePower);
            telemetry.addData("slide pos", slides.getPosition());

            //INTAKE
            if (gamepad1.y) {
                claw.setPower(1);
                intake.setIntakePower(0.6);
            } else if (gamepad1.b) {
                claw.setPower(-1);
                intake.setIntakePower(-1);
            } else {
                claw.setPower(0);
                intake.setIntakePower(0);
            }

            //CLIMBER
            climberPower = gamepad1.right_trigger - gamepad1.left_trigger;
            climber.setPower(climberPower);

            /*
            outtake.periodic();
            //pre-sets
            if (gamepad1.x) {
                claw.setPower(Constants.Claw.intake);
            } else if (gamepad1.y) {
                claw.setPower(Constants.Claw.outake);
            } else {
                claw.setPower(Constants.Claw.stop);
            }

        if (gamepad1.dpad_right) {
            outtake.presetSlides(Constants.Slides.medSlides);
        }
        if (gamepad1.dpad_up) {
            plane.setPos(Constants.Plane.release);
        } else {
            plane.setPos(Constants.Plane.hold);
        }
             */
        }
//        if (aprilTagDetectionPipeline.getDesiredTag(10).isPresent()) {
//            runnerPose10 = AprilTagToRoadRunner.tagToRunner(aprilTagDetectionPipeline.getDesiredTag(10).get());
//        }
//        if (aprilTagDetectionPipeline.getDesiredTag(9).isPresent()) {
//            runnerPose9 = AprilTagToRoadRunner.tagToRunner(aprilTagDetectionPipeline.getDesiredTag(9).get());
//        }

//        telemetry.addData("actual tag pos 10", runnerPose10);
//        aprilTagDetectionPipeline.telemetryAprilTag();
    }
    @Override
    public void stop() {
         claw.stopClaw();
         outtake.stop();
         intake.stopIntake();
         drivetrain.stopDrive();
         aprilTagDetectionPipeline.AprilTagStop();
    }
}
