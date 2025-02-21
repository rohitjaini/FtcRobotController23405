package org.firstinspires.ftc.teamcode.commandbase.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

@Disabled
@TeleOp(name = "OpenCV Testing")
public class openCVTesting implements VisionProcessor {

    private openCVTesting controlHubCam;
    private Mat hsvMat;
    private Mat mask;
    private Paint paint;
    private String detectedColor = "None";
    private OpMode opMode;

    public openCVTesting(OpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Initialization code here
        hsvMat = new Mat();
        mask = new Mat();
        paint = new Paint();
        paint.setColor(Color.RED);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(5);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Frame processing code here
        //ranges for yellow red and blue color detection
        Scalar lowerYellow = new Scalar(20, 100, 100);
        Scalar upperYellow = new Scalar(30, 255, 255);
        Scalar lowerRed1 = new Scalar(0, 100, 100);
        Scalar upperRed1 = new Scalar(10, 255, 255);
        Scalar lowerRed2 = new Scalar(160, 100, 100);
        Scalar upperRed2 = new Scalar(179, 255, 255);
        Scalar lowerBlue = new Scalar(100, 100, 100);
        Scalar upperBlue = new Scalar(140, 255, 255);

        Mat yellowMask = new Mat();
        Mat redMask1 = new Mat();
        Mat redMask2 = new Mat();
        Mat blueMask = new Mat();
        Core.inRange(hsvMat, lowerYellow, upperYellow, yellowMask);
        Core.inRange(hsvMat, lowerRed1, upperRed1, redMask1);
        Core.inRange(hsvMat, lowerRed2, upperRed2, redMask2);
        Core.inRange(hsvMat, lowerBlue, upperBlue, blueMask);

        Mat redMask = new Mat();
        Core.add(redMask1, redMask2, redMask);

        // determining color
        double yellowSample = Core.sumElems(yellowMask).val[0];
        double redSample = Core.sumElems(redMask).val[0];
        double blueSample = Core.sumElems(blueMask).val[0];

        if (yellowSample > redSample && yellowSample > blueSample) {
            detectedColor = "Yellow";
        } else if (redSample > yellowSample && redSample > blueSample) {
            detectedColor = "Red";
        } else if (blueSample > yellowSample && blueSample > redSample) {
            detectedColor = "Blue";
        } else {
            detectedColor = "None";
        }

        if (opMode != null) {
            opMode.telemetry.addData("Detected Color", detectedColor);
            opMode.telemetry.update();
        }

        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Drawing code here
        canvas.drawRect(50, 50, 200, 200, paint);
    }
}
//opMode.telemetry.addData("Detected Color", detectedColor);
//        opMode.telemetry.update();
//        controlHubCam.stopStreaming();