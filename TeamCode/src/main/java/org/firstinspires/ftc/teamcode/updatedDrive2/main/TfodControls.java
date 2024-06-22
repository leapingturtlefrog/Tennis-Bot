package org.firstinspires.ftc.teamcode.updatedDrive2.main;

import static org.firstinspires.ftc.teamcode.updatedDrive2.constants.Constants.LABELS;
import static org.firstinspires.ftc.teamcode.updatedDrive2.constants.Constants.TFOD_MODEL_FILE;
import static org.firstinspires.ftc.teamcode.updatedDrive2.storage.Positions.currentMode;
import static org.firstinspires.ftc.teamcode.updatedDrive2.storage.Positions.currentState;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.updatedDrive2.storage.Positions;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.math.BigDecimal;
import java.util.List;

public class TfodControls extends LinearOpMode {
    public VisionPortal visionPortal;

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
        myTfodProcessor.setMinResultConfidence((float) 0.75); //0.40 //.10 //lower for when at court

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

        /* WRONG, does y value based on distance
        //approximates the distance based on the y value
        //552 + -3.69x + 0.0245x^2 + -7.77E-05x^3 + 9.57E-08x^4
        savedDistance = 552 - 3.69*savedY + 0.0245*savedY*savedY - 0.0000777*Math.pow(savedY, 3) + 0.0000000957*Math.pow(savedY, 4);*/

        //Correct, produces distance based on y value //update: produces near-zero values
        //1.31E+07 + -214233x + 1461x^2 + -5.31x^3 + 0.0109x^4 + -1.18E-05x^5 + 5.36E-09x^6
        //savedDistance = 13100000 - 214233*savedY + 1461*savedY*savedY - 5.31*Math.pow(savedY, 3) + 0.0109*Math.pow(savedY, 4) - 0.000018*Math.pow(savedY, 5) + 0.00000000536*Math.pow(savedY, 6);

        //212101 + -2265x + 9.08x^2 + -0.0162x^3 + 1.08E-05x^4
        //savedDistance = 212101 - 2265*savedY + 9.08*savedY*savedY - 0.0162*Math.pow(savedY, 3) + 0.0000108*Math.pow(savedY, 4);

        /*
        BigDecimal y = new BigDecimal(savedY);

        BigDecimal a1 = new BigDecimal(212101);

        BigDecimal a2a = new BigDecimal(-2265);
        BigDecimal a2 = a2a.multiply(y);

        BigDecimal a3a = new BigDecimal(9.08);
        BigDecimal a3b = y.pow(2);
        BigDecimal a3 = a3a.multiply(a3b);

        BigDecimal a4a = new BigDecimal(-0.0162);
        BigDecimal a4b = y.pow(3);
        BigDecimal a4 = a4a.multiply(a4b);

        BigDecimal a5a = new BigDecimal(0.0000108);
        BigDecimal a5b = y.pow(4);
        BigDecimal a5 = a5a.multiply(a5b);

        BigDecimal ans = a1.add(a2.add(a3.add(a4.add(a5))));

        savedDistance = ans.doubleValue();*/

        //ab^y + c WRONG
        //a=239.735, b=0.983259, c=315.151
        //savedDistance = 239.735*Math.pow(0.983259, savedY) + 315.151;


        //ab^y + c
        //a=7.5252e7, b=0.960118, c=60.4614
        savedDistance = (7.5252E7) * (Math.pow(0.960118, savedY)) + 60.4614;

    }

}
