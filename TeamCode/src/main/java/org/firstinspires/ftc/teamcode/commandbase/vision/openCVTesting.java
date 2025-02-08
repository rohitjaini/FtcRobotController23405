package org.firstinspires.ftc.teamcode.commandbase.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

public class openCVTesting implements VisionProcessor {

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Initialization code here
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Frame processing code here
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Drawing code here
    }
}