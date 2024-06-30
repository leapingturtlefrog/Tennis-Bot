package org.firstinspires.ftc.teamcode.copyThatMayChange.opmode;

import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.LABELS;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.TFOD_MODEL_FILE;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.confidenceLevel1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;


@TeleOp(name="A.Webcam_Recognition_Tfod", group = "A")
//@Disabled
public class Webcam_Recognition_Tfod extends LinearOpMode {

    TfodProcessor tfodProcessor1;

    VisionPortal visionPortal1;

    @Override
    public void runOpMode() throws InterruptedException {
        TfodProcessor.Builder myTfodProcessorBuilder = new TfodProcessor.Builder();

        tfodProcessor1 = myTfodProcessorBuilder.setUseObjectTracker(false)
                //.setMaxNumRecognitions(5)
                .setNumDetectorThreads(1)
                .setNumExecutorThreads(2)
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .build();

        tfodProcessor1.setMinResultConfidence(confidenceLevel1);

        visionPortal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tfodProcessor1)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .build();

        while (visionPortal1.getCameraState() != VisionPortal.CameraState.STREAMING)
        {
            telemetry.addLine("Waiting for portal 1 to come online");

        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            telemetry.addData("Number of recognitions", tfodProcessor1.getRecognitions().size());

            sleep(50);

        }

    }

}
