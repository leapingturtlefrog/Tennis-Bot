package org.firstinspires.ftc.teamcode.copyThatMayChange.main;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.LABELS;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.TFOD_MODEL_FILE;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.confidenceLevel1;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.confidenceLevel2;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.maxExposure1;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.maxExposure2;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.maxGain1;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.maxGain2;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.minExposure1;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.minExposure2;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.minGain1;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.minGain2;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;

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

public class CameraControls {

    public List<Recognition> currentRecognitions1, currentRecognitions2;

    //for the recognitions we are heading to
    public int[] currentDetectionIndex = {-1, -1};
    public static List<Recognition> savedRecognitions;
    public static int[] savedDetectionIndex;
    public static double savedHeadingError;
    public static double savedDistance;
    public static double savedX, savedY;

    public TfodProcessor tfodProcessor1, tfodProcessor2;
    public AprilTagProcessor aprilTagProcessor1, aprilTagProcessor2;
    public VisionPortal visionPortal1, visionPortal2;

    public CameraControls(HardwareMap hardwareMap) throws InterruptedException {
        //initialize the tfod and apriltag for both vision portals
        initCameras(hardwareMap);
        //get the min and max gain and exposure for both cameras
        getCameraSetting();

        TelemetryControls.add("All cameras online");

    }

    private void initCameras(HardwareMap hardwareMap) throws InterruptedException {
        TfodProcessor.Builder myTfodProcessorBuilder = new TfodProcessor.Builder();

        tfodProcessor1 = myTfodProcessorBuilder.setUseObjectTracker(false)
                //.setMaxNumRecognitions(5)
                .setNumDetectorThreads(1)
                .setNumExecutorThreads(2)
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .build();

        tfodProcessor1.setMinResultConfidence(confidenceLevel1);

        //MUST HAVE USE OBJECT TRACKER FALSE for two cameras
        tfodProcessor2 = myTfodProcessorBuilder.setUseObjectTracker(false)
                //.setMaxNumRecognitions(5)
                .setNumDetectorThreads(1)
                .setNumExecutorThreads(2)
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .build();

        tfodProcessor2.setMinResultConfidence(confidenceLevel2);
        //myTfodProcessor2.setClippingMargins(0, 0, 0, 90);

        //

        AprilTagProcessor.Builder myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();

        aprilTagProcessor1 = myAprilTagProcessorBuilder.build();
        aprilTagProcessor1.setDecimation(1);

        aprilTagProcessor2 = myAprilTagProcessorBuilder.build();
        aprilTagProcessor2.setDecimation(1);

        //

        int[] viewIDs = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        visionPortal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tfodProcessor1)
                .addProcessor(aprilTagProcessor1)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setLiveViewContainerId(viewIDs[0])
                .build();

        //visionPortal1.setProcessorEnabled(aprilTagProcessor1, false);

        while (visionPortal1.getCameraState() != VisionPortal.CameraState.STREAMING)
        {
            TelemetryControls.add("Waiting for portal 1 to come online");

        }

        //setManualExposure(exposure1, gain1, visionPortal1);

        visionPortal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .addProcessor(tfodProcessor2)
                .addProcessor(aprilTagProcessor2)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setLiveViewContainerId(viewIDs[1])
                .build();

        //visionPortal2.setProcessorEnabled(aprilTagProcessor2, false);

        while (visionPortal2.getCameraState() != VisionPortal.CameraState.STREAMING)
        {
            TelemetryControls.add("Waiting for portal 2 to come online");

        }

        sleep(500);

        //setManualExposure(exposure2, gain2, visionPortal2);

    }

    public void setManualExposure(int exposureMS, int gain, VisionPortal visionPortal) throws InterruptedException {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            TelemetryControls.add("Camera", "Waiting");

            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                sleep(20);
            }
            TelemetryControls.add("Camera", "Ready");

        }

        // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        sleep(20);

        // Set Gain.
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        sleep(20);

    }

    public void updateTfod() {


    }

    public void updateAprilTags() {


    }

    public void saveRecognitionData() {
        //left camera
        if (currentDetectionIndex[0] == 0) {
            savedRecognitions = currentRecognitions1;
            savedDetectionIndex = currentDetectionIndex;

            savedHeadingError = -savedRecognitions.get(savedDetectionIndex[1]).estimateAngleToObject(AngleUnit.DEGREES) + 0;//TODO: change angle

            //right camera
        } else if (currentDetectionIndex[1] == 1) {
            savedRecognitions = currentRecognitions2;
            savedDetectionIndex = currentDetectionIndex;


            savedHeadingError = -savedRecognitions.get(savedDetectionIndex[1]).estimateAngleToObject(AngleUnit.DEGREES) - 0;//TODO: change angle

        }


        savedX = (savedRecognitions.get(savedDetectionIndex[1]).getLeft() + savedRecognitions.get(savedDetectionIndex[1]).getRight()) / 2.0;
        savedY = (savedRecognitions.get(savedDetectionIndex[1]).getTop() + savedRecognitions.get(savedDetectionIndex[1]).getBottom()) / 2.0;

        //ab^y + c
        //a=7.5252e7, b=0.960118, c=60.4614
        savedDistance = (7.5252E7) * (Math.pow(0.960118, savedY)) + 60.4614;

    }

    private void getCameraSetting() throws InterruptedException {
        // Ensure Vision Portal has been setup.
        if (visionPortal1 == null) {
            return;
        }

        // Wait for the camera to be open
        if (visionPortal1.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera1", "Waiting");
            telemetry.update();
            while (visionPortal1.getCameraState() != VisionPortal.CameraState.STREAMING) {
                sleep(20);
            }
            telemetry.addData("Camera1", "Ready");
            telemetry.update();
        }

        // Get camera control values unless we are stopping.
        ExposureControl exposureControl = visionPortal1.getCameraControl(ExposureControl.class);
        minExposure1 = (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
        maxExposure1 = (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal1.getCameraControl(GainControl.class);
        minGain1 = gainControl.getMinGain();
        maxGain1 = gainControl.getMaxGain();

        //

        if (visionPortal2 == null) {
            return;
        }

        // Wait for the camera to be open
        if (visionPortal2.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera2", "Waiting");
            telemetry.update();
            while (visionPortal2.getCameraState() != VisionPortal.CameraState.STREAMING) {
                sleep(20);
            }
            telemetry.addData("Camera2", "Ready");
            telemetry.update();
        }

        // Get camera control values unless we are stopping.
        exposureControl = visionPortal2.getCameraControl(ExposureControl.class);
        minExposure2 = (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
        maxExposure2 = (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

        gainControl = visionPortal1.getCameraControl(GainControl.class);
        minGain2 = gainControl.getMinGain();
        maxGain2 = gainControl.getMaxGain();
    }

}
