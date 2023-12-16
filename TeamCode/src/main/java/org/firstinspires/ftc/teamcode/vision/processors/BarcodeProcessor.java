package org.firstinspires.ftc.teamcode.vision.processors;

import android.annotation.SuppressLint;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Rect;

import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utility.BarcodePosition;
import org.firstinspires.ftc.teamcode.utility.RobotSide;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Optional;

public class BarcodeProcessor implements VisionProcessor {

    private final Mat hue = new Mat();
    private final Mat saturation = new Mat();
    private final Mat value = new Mat();
    private final ArrayList<Mat> channels = new ArrayList<Mat>();
    private final Mat redtmp1 = new Mat();
    private final Mat redtmp2 = new Mat();
    private final Mat hsv = new Mat();
    private final Mat thresholded = new Mat();
    public RobotSide side = RobotSide.Red;
    private Mat cropped = new Mat();
    private int leftNumber = 0;
    private final int middleNumber = 0;
    private int rightNumber = 0;
    private int swapNumber = 0;
    private Mat leftHalf = new Mat();
    private final Mat middleThird = new Mat();
    private Mat rightHalf = new Mat();
    private int xPadding;
    private int yPadding;
    private double percentDiff;
    private double percentDiff3;
    private Optional<BarcodePosition> lastKnownPosition = Optional.empty();
    public void setSide(RobotSide side){
        this.side = side;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        channels.add(hue);
        channels.add(saturation);
        channels.add(value);
    }

    public Optional<BarcodePosition> getLastKnownPosition() {
        return lastKnownPosition;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        // Specifically look at only values in the HSV scheme
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Crop the padding around the actual barcodes
        xPadding = input.cols() / 4;
        yPadding = 24;

        cropped = hsv.submat(
                xPadding,
                input.rows() - xPadding, yPadding, input.cols() - yPadding);

        //
        Core.split(cropped, channels);

        if (side == RobotSide.Red) {
            // Here we have to split the red into two mats,
            // one for the values above 1 and one for the values below 1

            // this helps with understanding HSV
            // https://web.cs.uni-paderborn.de/cgvb/colormaster/web/color-systems/hsv.html
            Core.inRange(cropped, Constants.Vision.redPosMin, Constants.Vision.redPosMax, redtmp1);
            Core.inRange(cropped, Constants.Vision.redNegMin, Constants.Vision.redNegMax, redtmp2);

            Core.bitwise_or(redtmp1, redtmp2, thresholded);
        } else {
            // Extract only blue values
            Core.inRange(cropped, Constants.Vision.blueMin, Constants.Vision.blueMax, thresholded);
        }

        // Split the camera view into 2
        int columnSize = thresholded.cols() / 2;
        leftHalf = thresholded.colRange(0, columnSize);
        // middleThird = thresholded.colRange(columnSize, 2 * columnSize);
        rightHalf = thresholded.colRange(columnSize, thresholded.cols());

        // Count every pixel that has target color
        leftNumber = Core.countNonZero(leftHalf);
        //middleNumber = Core.countNonZero(middleThird);
        rightNumber = Core.countNonZero(rightHalf);

        // Reverse left and right if red or blue
        if (side == RobotSide.Red) {
            swapNumber = rightNumber;
            rightNumber = leftNumber;
            leftNumber = swapNumber;
        }

        // Prevent divide by zero error
       // if (leftNumber == 0 || rightNumber == 0) return thresholded;
        leftNumber += 1;
        rightNumber += 1;
        percentDiff = (leftNumber - rightNumber) / rightNumber;
        percentDiff3 = (rightNumber - leftNumber)/leftNumber;
        if ((Math.abs(percentDiff) < Constants.Vision.percentThreshold )&&(Math.abs(percentDiff3) < Constants.Vision.percentThreshold )) {
            lastKnownPosition = Optional.of(org.firstinspires.ftc.teamcode.utility.BarcodePosition.One);
        } else if (leftNumber > rightNumber) {
            lastKnownPosition = Optional.of(org.firstinspires.ftc.teamcode.utility.BarcodePosition.Two);
        } else {
            lastKnownPosition = Optional.of(org.firstinspires.ftc.teamcode.utility.BarcodePosition.Three);
        }

        //        if (leftNumber > middleNumber && leftNumber > rightNumber)
//            lastKnownPosition = Optional.of(org.firstinspires.ftc.teamcode.utility.BarcodePosition.One);
//        if (middleNumber > leftNumber && middleNumber > rightNumber)
//            lastKnownPosition = Optional.of(org.firstinspires.ftc.teamcode.utility.BarcodePosition.Two);
//        if (rightNumber > leftNumber && rightNumber > middleNumber)
//            lastKnownPosition = Optional.of(org.firstinspires.ftc.teamcode.utility.BarcodePosition.Three);

        return thresholded;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint whiteTextPaint = new Paint();
        whiteTextPaint.setTextSize(50);
        whiteTextPaint.setARGB(255, 255, 255, 255);
        Bitmap b = Bitmap.createBitmap(thresholded.width(), thresholded.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(thresholded, b);
        canvas.drawBitmap(b, null, new Rect(0, 0, thresholded.width(), thresholded.height()), null);
//        canvas.drawText(String.format("Halves: %d %d", leftNumber, rightNumber), 10, 50, whiteTextPaint);
        canvas.drawText(String.format("Percent Diff: %f", percentDiff3), 20, 50, whiteTextPaint);
    }
}

