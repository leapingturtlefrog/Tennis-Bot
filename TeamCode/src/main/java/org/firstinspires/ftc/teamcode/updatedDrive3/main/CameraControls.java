package org.firstinspires.ftc.teamcode.updatedDrive3.main;

import static org.firstinspires.ftc.teamcode.updatedDrive3.constants.Constants.LABELS;
import static org.firstinspires.ftc.teamcode.updatedDrive3.constants.Constants.TFOD_MODEL_FILE;
import static org.firstinspires.ftc.teamcode.updatedDrive3.constants.Constants.exposure;
import static org.firstinspires.ftc.teamcode.updatedDrive3.constants.Constants.gain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class CameraControls extends LinearOpMode {
    public List<Recognition> currentRecognitions;
    public int currentDetectionIndex = -1;
    //for the recognitions we are heading to
    public static List<Recognition> savedRecognitions;
    public static int savedDetectionIndex;
    public static double savedHeadingError;
    public static double savedDistance;
    public static double savedX, savedY;

    public HardwareMap hardwareMap;

    public AprilTagProcessor myAprilTagProcessor;
    public TfodProcessor myTfodProcessor;
    public VisionPortal myVisionPortal;

    public static int     myExposure  ;
    public static int     minExposure ;
    public static int     maxExposure ;
    public static int     myGain      ;
    public static int     minGain ;
    public static int     maxGain ;


    public void runOpMode() throws InterruptedException {}

    public CameraControls(HardwareMap hwMap) {
        hardwareMap = hwMap;

        initTfod();

        //sets the camera exposure (0-1000) and gain (0-255) (for this webcam, for others use OptimizeExposure
        //opmode in the external samples to find the values
        getCameraSetting();
        myExposure = Math.max(exposure, minExposure);
        myGain = Math.min(gain, maxGain);
        setManualExposure(myExposure, myGain);

    }

    public void initTfod() {
        VisionPortal.Builder myVisionPortalBuilder;

        //tfod
        TfodProcessor.Builder myTfodProcessorBuilder;

        // First, create a TfodProcessor.
        // Create a new TfodProcessor.Builder object.
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        // Set the name of the file where the model can be found.
        myTfodProcessorBuilder.setModelFileName(TFOD_MODEL_FILE);
        // Set the full ordered list of labels the model is trained to recognize.
        myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith(LABELS));
        // Build the TensorFlow Object Detection processor and assign it to a variable.
        myTfodProcessor = myTfodProcessorBuilder.build();
        // Set the minimum confidence at which to keep recognitions.
        myTfodProcessor.setMinResultConfidence((float) 0.70); //0.40 //.10 //lower for when at court

        //aprilTag
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;

        // Create the AprilTag processor by using a builder.
        // Create a new AprilTagProcessor.Builder object and assign it to a variable.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        // Build the AprilTag processor and assign it to a variable.
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        // Set the detector decimation.
        myAprilTagProcessor.setDecimation(1);


        myVisionPortalBuilder = new VisionPortal.Builder();

        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Add the AprilTag processor.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        // Add the TensorFlow processor.
        myVisionPortalBuilder.addProcessor(myTfodProcessor);
        // Build the VisionPortal object and assign it to a variable.
        myVisionPortal = myVisionPortalBuilder.build();

        sleep(20);

    }

    /*
    //initialize the object detection
    public void initTfod() {
        //create new TensorFlow processor
        tfod = new TfodProcessor.Builder()
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)

                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.enableLiveView(true);
        //builder.setLiveViewContainerId(1);
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.setAutoStopLiveView(false);
        //builder.addProcessor(tfod);
        builder.addProcessors(tfod, AprilTagProcessor.easyCreateWithDefaults());

        visionPortal = builder.build();


        //the confidence interval for objects to be recognized
        tfod.setMinResultConfidence(0.10f);
    }*/

    //Add the telemetry for the object detection and the highest confidence index
    public void updateTfod() {
        currentRecognitions = myTfodProcessor.getRecognitions();

        telemetry.addData("# objects detected", currentRecognitions.size());

        double highestConfidence = 0;
        int index = 0;
        currentDetectionIndex = -1;

        //display info on each recognition
        for (Recognition recognition : currentRecognitions) {
            if (recognition.getConfidence() > highestConfidence) {
                highestConfidence = recognition.getConfidence();
                currentDetectionIndex = index;

            }

            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% conf.)",
                    recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f, %.0f", x, y);
            telemetry.addData("Angle", recognition.estimateAngleToObject(AngleUnit.DEGREES));
            telemetry.addData("- Size", "%.0f x %.0f",
                    recognition.getWidth(), recognition.getHeight());

            index++;
        }
    }

    //save the recognition data for the highest confidence recognition
    public void saveRecognitionData() {
        savedRecognitions = currentRecognitions;

        savedDetectionIndex = currentDetectionIndex;
        savedHeadingError = -savedRecognitions.get(savedDetectionIndex).estimateAngleToObject(AngleUnit.DEGREES);

        savedX = (savedRecognitions.get(savedDetectionIndex).getLeft() + savedRecognitions.get(savedDetectionIndex).getRight()) / 2.0;
        savedY = (savedRecognitions.get(savedDetectionIndex).getTop() + savedRecognitions.get(savedDetectionIndex).getBottom()) / 2.0;

        //ab^y + c
        //a=7.5252e7, b=0.960118, c=60.4614
        savedDistance = (7.5252E7) * (Math.pow(0.960118, savedY)) + 60.4614;

    }

    public boolean    setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (myVisionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (!isStopRequested() && (myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = myVisionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);

            // Set Gain.
            GainControl gainControl = myVisionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            return (true);
        } else {
            return (false);
        }
    }

    /*
        Read this camera's minimum and maximum Exposure and Gain settings.
        Can only be called AFTER calling initAprilTag();
     */
    private void getCameraSetting() {
        // Ensure Vision Portal has been setup.
        if (myVisionPortal == null) {
            return;
        }

        // Wait for the camera to be open
        if (myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (!isStopRequested() && (myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
        }

        // Get camera control values unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = myVisionPortal.getCameraControl(ExposureControl.class);
            minExposure = (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposure = (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

            GainControl gainControl = myVisionPortal.getCameraControl(GainControl.class);
            minGain = gainControl.getMinGain();
            maxGain = gainControl.getMaxGain();
        }
    }

}
