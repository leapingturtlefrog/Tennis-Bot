package org.firstinspires.ftc.teamcode.updatedDrive.main;

import static org.firstinspires.ftc.teamcode.updatedDrive.constants.Constants.LABELS;
import static org.firstinspires.ftc.teamcode.updatedDrive.constants.Constants.TFOD_MODEL_FILE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class TfodControls extends LinearOpMode {
    public VisionPortal visionPortal;

    public List<Recognition> currentRecognitions;
    public int currentDetectionIndex = -1;
    //for the recognitions we are heading to
    public List<Recognition> savedRecognitions;
    public int savedDetectionIndex;
    public double savedHeadingError;
    public double savedDistance;
    public double savedX, savedY;

    public HardwareMap hardwareMap;

    public AprilTagProcessor myAprilTagProcessor;
    public TfodProcessor myTfodProcessor;
    public VisionPortal myVisionPortal;


    public void runOpMode() throws InterruptedException {}

    public TfodControls(HardwareMap hwMap) {
        hardwareMap = hwMap;

        initTfod();

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
        myTfodProcessor.setMinResultConfidence((float) 0.10);

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
        int index = -1;
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


}
