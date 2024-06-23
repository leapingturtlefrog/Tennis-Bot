package org.firstinspires.ftc.teamcode.updatedDrive4.examples;

import static org.firstinspires.ftc.teamcode.updatedDrive4.constants.Constants.TFOD_MODEL_FILE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.concurrent.TimeUnit;

@TeleOp
public class Multi4 extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        TfodProcessor myTfodProcessor, myTfodProcessor2;

        TfodProcessor.Builder myTfodProcessorBuilder = new TfodProcessor.Builder();

        myTfodProcessor = myTfodProcessorBuilder.setUseObjectTracker(false)
                .setMaxNumRecognitions(1)
                .setNumDetectorThreads(1)
                .setNumExecutorThreads(1)
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(new String[]{""})
                .build();

        myTfodProcessor.setMinResultConfidence((float) 0.10);

        myTfodProcessor2 = myTfodProcessorBuilder.setUseObjectTracker(false)
                .setMaxNumRecognitions(1)
                .setNumDetectorThreads(1)
                .setNumExecutorThreads(1)
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(new String[]{""})
                .build();

        myTfodProcessor2.setMinResultConfidence((float) 0.10);

        //

        telemetry.setMsTransmissionInterval(50);

        int[] viewIDs = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        VisionPortal portal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(myTfodProcessor)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setLiveViewContainerId(viewIDs[0])
                .build();

        while (!isStopRequested() && portal1.getCameraState() != VisionPortal.CameraState.STREAMING)
        {
            telemetry.addLine("Waiting for portal 1 to come online");
            telemetry.update();
        }

        setManualExposure(7, 255, portal1);

        if (isStopRequested())
        {
            return;
        }

        VisionPortal portal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .addProcessor(myTfodProcessor2)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setLiveViewContainerId(viewIDs[1])
                .build();

        while (!isStopRequested() && portal2.getCameraState() != VisionPortal.CameraState.STREAMING)
        {
            telemetry.addLine("Waiting for portal 2 to come online");
            telemetry.update();
        }

        setManualExposure(60, 255, portal2);

        if (isStopRequested())
        {
            return;
        }

        telemetry.addLine("All cameras online");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested())
        {
            int exp = (int) Math.abs((gamepad1.left_stick_y + gamepad1.left_trigger) * 100);
            //19
            setManualExposure(exp, 255, portal2);

            telemetry.addLine("All cameras online");
            telemetry.addData("exp", exp);
            telemetry.update();
            sleep(500);
        }
    }

    private boolean setManualExposure(int exposureMS, int gain, VisionPortal visionPortal) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
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
            return (true);
        } else {
            return (false);
        }
    }
}