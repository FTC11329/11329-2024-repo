package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystems.Cameras;
import org.firstinspires.ftc.teamcode.subsystems.ClawSensor;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.SpecialIntake;
import org.firstinspires.ftc.teamcode.utility.BarcodePosition;

@Autonomous(name = "Red Left Center CRI", group = "Testing")
@Config
public class CRIRedLeftToRightBDThrCenter extends OpMode {
    boolean whiteLeft;
    static Pose2d startingPose = new Pose2d(-65.25, -63, Math.toRadians(90));
    static Vector2d finalPlacePos2 = new Vector2d(70, -10);

    static Pose2d pickupSpecial = new Pose2d(-16.5, -13, Math.toRadians(135));
    static Pose2d pickupSpecial2 = new Pose2d(-15,-10, Math.toRadians(135));

    TrajectorySequenceBuilder placeSpikeMark1 = null;
    TrajectorySequenceBuilder placeSpikeMark2 = null;
    TrajectorySequenceBuilder placeSpikeMark3 = null;

    TrajectorySequenceBuilder restOfIt = null;

    Intake intake;
    Lights lights;
    Outtake outtake;
    Cameras cameras;
    ClawSensor clawSensor;
    Drivetrain drivetrain;
    SpecialIntake specialIntake;
    DistanceSensors distanceSensors;

    ConstantCRIPathsRed constantCRIPaths;
    ConstantCRIPathsRed.PlacePurplePaths placePurplePathsRed;
    ConstantCRIPathsRed.PickupWhitePixelStack pickupWhitePixelStack;
    ConstantCRIPathsRed.PlaceOnBackDrop placeOnBackDrop;


    public void init() {
        lights = new Lights(hardwareMap);
        lights.setDumbLed(1);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        cameras = new Cameras(hardwareMap);
        clawSensor = new ClawSensor(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        specialIntake = new SpecialIntake(hardwareMap);
        distanceSensors = new DistanceSensors(hardwareMap);
        outtake.holdBackClaw(true);

        constantCRIPaths = new ConstantCRIPathsRed(telemetry, intake, outtake, cameras, clawSensor, drivetrain, specialIntake, pickupSpecial, pickupSpecial2, finalPlacePos2);
        placePurplePathsRed = constantCRIPaths.placePurplePathsRed;
        pickupWhitePixelStack = constantCRIPaths.pickupWhitePixelStack;
        placeOnBackDrop = constantCRIPaths.placeOnBackDrop;

        //1**************************************************************************
        placeSpikeMark1 = drivetrain.trajectorySequenceBuilder(startingPose);
        placePurplePathsRed.LeftPlacePos1Center.run(placeSpikeMark1);
        //2**************************************************************************
        placeSpikeMark2 = drivetrain.trajectorySequenceBuilder(startingPose);
        placePurplePathsRed.LeftPlacePos2Center.run(placeSpikeMark2);
        //3**************************************************************************
        placeSpikeMark3 = drivetrain.trajectorySequenceBuilder(startingPose);
        placePurplePathsRed.LeftPlacePos3Center.run(placeSpikeMark3);
        lights.setDumbLed(0);
    }

    @Override
    public void init_loop() {
        boolean isBack = gamepad1.a;
        cameras.setCameraSide(isBack);

        BarcodePosition barcodePosition = distanceSensors.getDirectionRed(true);
        telemetry.addData("Barcode Position", barcodePosition);
        telemetry.addData("FPS", cameras.switchingCamera.getFps());
        telemetry.addData("Is back", isBack);
        telemetry.addData("Front color", clawSensor.getFrontColor());
        telemetry.addData("Back Color" , clawSensor.getBackColor());
        telemetry.update();
    }

    @Override
    public void start() {
        cameras.setCameraSideThreaded(true);
        BarcodePosition barcodePosition = distanceSensors.getDirectionRed(true);

        drivetrain.getPoseEstimateOptical(startingPose);

        TrajectorySequence placeSpikeMarkActual = null;

        if (barcodePosition == BarcodePosition.One) {
            placeSpikeMarkActual = placeSpikeMark1.build();
        } else if (barcodePosition == BarcodePosition.Two) {
            placeSpikeMarkActual = placeSpikeMark2.build();
        } else {
            placeSpikeMarkActual = placeSpikeMark3.build();
        }

        drivetrain.followTrajectorySequence(placeSpikeMarkActual);
        restOfIt = drivetrain.trajectorySequenceBuilder(placeSpikeMarkActual.end());
        pickupWhitePixelStack.Left1stToCenterStack.run(restOfIt);


        if (barcodePosition == BarcodePosition.One) {
            placeOnBackDrop.CenterStackTo1stPlacePos.run(restOfIt);
        } else if (barcodePosition == BarcodePosition.Two) {
            placeOnBackDrop.CenterStackTo2ndPlacePos.run(restOfIt);
        } else {
            placeOnBackDrop.CenterStackTo3rdPlacePos.run(restOfIt);

        }


        pickupWhitePixelStack.BackDropToCenterStack.run(restOfIt);
        placeOnBackDrop.CenterStackToCenterBD.run(restOfIt);
        restOfIt
                .waitSeconds(10);

        drivetrain.followTrajectorySequenceAsync(restOfIt.build());
    }

    @Override
    public void loop() {
        drivetrain.update();
        outtake.periodic();
        if (clawSensor.autoSense()) {
            intake.setIntakePower(Constants.Intake.outake, 0);
        }
    }
}
