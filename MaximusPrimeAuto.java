package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name="MaximusPrimeAuto", group="OpMode")
public class MaximusPrimeAuto extends LinearOpMode {
    MaximusPrimeBase base;
    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;
    ElapsedTime cameraTimer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        base = new MaximusPrimeBase(this);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override public void onOpened() {webcam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT); }
            @Override public void onError(int errorCode) { }
        });

        while (pipeline.getAnalysis() == 0 && cameraTimer.seconds()<7) {
            base.tmrTeleop.seconds();
        }
        base.RetrieveShippingElementPosition();
        while (base.IsInitialized()) {
            base.AllianceDetermination();
            telemetry.addData("Blue", SkystoneDeterminationPipeline.blueSquareActivated);
            telemetry.addData("Green", SkystoneDeterminationPipeline.greenSquareActivated);
            telemetry.addData("Blue?", pipeline.avg1);
            telemetry.addData("Green?", pipeline.avg2);
            telemetry.update();
        }
        waitForStart();
        base.RetrieveShippingElementPosition();
        base.Autonomous();
        while (opModeIsActive()) {
            base.tmrTeleop.seconds();
        }
    }
    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
        static boolean blueSquareActivated = false;
        static boolean greenSquareActivated = false;
        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        /*
         * The core values which define the location and size of the sample regions
         */
        //x:130 y:210
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(130 ,100);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(130 ,290);
        // width: 40 height: 40
        static final int REGION_WIDTH = 55;
        static final int REGION_HEIGHT = 55; //65
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y - REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y - REGION_HEIGHT);
        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat region2_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;
        int avg2;
        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }
        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
        }
        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE , // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            if (avg1>140) {
                greenSquareActivated = true;
                blueSquareActivated = false;
            } else if (avg2>140) {
                blueSquareActivated = true;
                greenSquareActivated = false;
            } else {
                greenSquareActivated = false;
                blueSquareActivated = false;
            }
            return input;
        }
        public int getAnalysis()
        {
            return avg1;
        }
    }
}