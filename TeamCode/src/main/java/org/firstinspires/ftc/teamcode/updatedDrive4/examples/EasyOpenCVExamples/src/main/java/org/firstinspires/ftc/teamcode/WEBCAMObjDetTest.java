/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.updatedDrive4.examples.EasyOpenCVExamples.src.main.java.org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * In this sample, we demonstrate how to use the {@link OpenCvPipeline#onViewportTapped()}
 * callback to switch which stage of a pipeline is rendered to the viewport for debugging
 * purposes. We also show how to get data from the pipeline to your OpMode.
 */
@TeleOp
public class WEBCAMObjDetTest extends LinearOpMode
{
    OpenCvCamera phoneCam;
    StageSwitchingPipeline stageSwitchingPipeline;

    /*
// remember: H ranges 0-180, S and V range 0-255
            Scalar minValues = new Scalar(48, 64,
                    50);
            Scalar maxValues = new Scalar(127, 183,
                    250);

             */
    static int minH = 48, minS = 64, minV = 50, maxH = 127, maxS = 183, maxV = 250;

    //31, 144, 134, 154, 255, 255

    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        stageSwitchingPipeline = new StageSwitchingPipeline();
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.setPipeline(stageSwitchingPipeline);
                phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            /*
// remember: H ranges 0-180, S and V range 0-255
            Scalar minValues = new Scalar(48, 64,
                    50);
            Scalar maxValues = new Scalar(127, 183,
                    250);

             */
            if (gamepad1.left_stick_x > 0.1) {
                maxH = Math.round(gamepad1.left_stick_x * 180);
            } else if (gamepad1.left_stick_x < -0.1) {
                minH = Math.round(-gamepad1.left_stick_x * 180);
            }

            if (gamepad1.right_stick_y > 0.1) {
                maxV = Math.round(gamepad1.right_stick_y * 255);
            } else if (gamepad1.right_stick_y < -0.1) {
                minV = Math.round(-gamepad1.right_stick_y * 255);
            }

            if (gamepad1.right_trigger > 0.1) {
                maxS = Math.round(gamepad1.right_trigger * 255);
            } else if (gamepad1.left_trigger > 0.1) {
                minS = Math.round(gamepad1.left_trigger * 255);
            }
            telemetry.addData("Num contours found", stageSwitchingPipeline.getNumContoursFound());
            telemetry.addData("HSV mins", "%d : %d : %d", minH, minS, minV);
            telemetry.addData("HSV maxes", "%d : %d : %d", maxH, maxS, maxV);
            telemetry.update();
            sleep(100);
        }
    }

    /*
     * With this pipeline, we demonstrate how to change which stage of
     * is rendered to the viewport when the viewport is tapped. This is
     * particularly useful during pipeline development. We also show how
     * to get data from the pipeline to your OpMode.
     */
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat blurredImage = new Mat();
        Mat hsvImage = new Mat();
        Mat mask = new Mat();
        Mat morphOutput = new Mat();

        //

        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat contoursOnFrameMat = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();
        int numContoursFound;

        enum Stage
        {
            TEST,
            YCbCr_CHAN2,
            THRESHOLD,
            CONTOURS_OVERLAYED_ON_FRAME,
            RAW_IMAGE,
        }

        private Stage stageToRenderToViewport = Stage.TEST;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {

            // remove some noise
            Imgproc.blur(input, blurredImage, new Size(7, 7));

// convert the frame to HSV
            Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_BGR2HSV);

// get thresholding values from the UI
// remember: H ranges 0-180, S and V range 0-255
            Scalar minValues = new Scalar(minH, minS, minV);
            Scalar maxValues = new Scalar(maxH, maxS, maxV);

// threshold HSV image to select tennis balls
            Core.inRange(hsvImage, minValues, maxValues, mask);

            // morphological operators
// dilate with large element, erode with small ones
            Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24, 24));
            Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(12, 12));

            Imgproc.erode(mask, morphOutput, erodeElement);
            Imgproc.erode(mask, morphOutput, erodeElement);

            Imgproc.dilate(mask, morphOutput, dilateElement);
            Imgproc.dilate(mask, morphOutput, dilateElement);


            // init
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();

// find contours
            Imgproc.findContours(morphOutput, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
            numContoursFound = contours.size();
// if any contour exist...
            if (hierarchy.size().height > 0 && hierarchy.size().width > 0)
            {
                // for each contour, display it in blue
                for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0])
                {
                    Imgproc.drawContours(input, contours, idx, new Scalar(250, 0, 0));
                }
            }

            input.copyTo(contoursOnFrameMat);

            return contoursOnFrameMat;


/*
            contoursList.clear();

            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            numContoursFound = contoursList.size();
            input.copyTo(contoursOnFrameMat);
            Imgproc.drawContours(contoursOnFrameMat, contoursList, -1, new Scalar(0, 0, 255), 3, 8);

            switch (stageToRenderToViewport)
            {
                case YCbCr_CHAN2:
                {
                    return yCbCrChan2Mat;
                }

                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case CONTOURS_OVERLAYED_ON_FRAME:
                {
                    return contoursOnFrameMat;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }*/
        }

        public int getNumContoursFound()
        {
            return numContoursFound;
        }
    }
}
