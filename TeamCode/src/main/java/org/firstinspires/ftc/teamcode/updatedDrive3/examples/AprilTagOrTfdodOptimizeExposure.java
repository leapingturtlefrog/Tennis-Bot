/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.updatedDrive3.examples;

import static org.firstinspires.ftc.teamcode.updatedDrive3.constants.Constants.LABELS;
import static org.firstinspires.ftc.teamcode.updatedDrive3.constants.Constants.TFOD_MODEL_FILE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode determines the best Exposure for minimizing image motion-blur on a Webcam
 * Note that it is not possible to control the exposure for a Phone Camera, so if you are using a Phone for the Robot Controller
 * this OpMode/Feature only applies to an externally connected Webcam
 *
 * The goal is to determine the smallest (shortest) Exposure value that still provides reliable Tag Detection.
 * Starting with the minimum Exposure and maximum Gain, the exposure is slowly increased until the Tag is
 * detected reliably from the likely operational distance.
 *
 *
 * The best way to run this optimization is to view the camera preview screen while changing the exposure and gains.
 *
 * To do this, you need to view the RobotController screen directly (not from Driver Station)
 * This can be done directly from a RC phone screen (if you are using an external Webcam), but for a Control Hub you must either plug an
 * HDMI monitor into the Control Hub HDMI port, or use an external viewer program like ScrCpy (https://scrcpy.org/)
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */

@TeleOp(name="Optimize AprilTag Exposure", group = "Concept")
@Disabled
public class AprilTagOrTfdodOptimizeExposure extends LinearOpMode
{
    public AprilTagProcessor myAprilTagProcessor;
    public TfodProcessor myTfodProcessor;
    private VisionPortal visionPortal = null;        // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private int     myExposure  ;
    private int     minExposure ;
    private int     maxExposure ;
    private int     myGain      ;
    private int     minGain ;
    private int     maxGain ;

    boolean thisExpUp = false;
    boolean thisExpDn = false;
    boolean thisGainUp = false;
    boolean thisGainDn = false;

    boolean lastExpUp = false;
    boolean lastExpDn = false;
    boolean lastGainUp = false;
    boolean lastGainDn = false;
    @Override public void runOpMode()
    {
        // Initialize the Apriltag Detection process
        //initAprilTag();
        initTfod();

        // Establish Min and Max Gains and Exposure.  Then set a low exposure with high gain
        getCameraSetting();
        myExposure = Math.min(5, minExposure);
        myGain = maxGain;
        setManualExposure(myExposure, myGain);

        // Wait for the match to begin.
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addLine("Find lowest Exposure that gives reliable detection.");
            telemetry.addLine("Use Left bump/trig to adjust Exposure.");
            telemetry.addLine("Use Right bump/trig to adjust Gain.\n");

            // Display how many Tags Detected
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            int numTags = currentDetections.size();
            if (numTags > 0 )
                telemetry.addData("Tag", "####### %d Detected  ######", currentDetections.size());
            else
                telemetry.addData("Tag", "----------- none - ----------");

            telemetry.addData("Exposure","%d  (%d - %d)", myExposure, minExposure, maxExposure);
            telemetry.addData("Gain","%d  (%d - %d)", myGain, minGain, maxGain);
            telemetry.update();

            // check to see if we need to change exposure or gain.
            thisExpUp = gamepad1.left_bumper;
            thisExpDn = gamepad1.left_trigger > 0.25;
            thisGainUp = gamepad1.right_bumper;
            thisGainDn = gamepad1.right_trigger > 0.25;

            // look for clicks to change exposure
            if (thisExpUp && !lastExpUp) {
                myExposure = Range.clip(myExposure + 1, minExposure, maxExposure);
                setManualExposure(myExposure, myGain);
            } else if (thisExpDn && !lastExpDn) {
                myExposure = Range.clip(myExposure - 1, minExposure, maxExposure);
                setManualExposure(myExposure, myGain);
            }

            // look for clicks to change the gain
            if (thisGainUp && !lastGainUp) {
                myGain = Range.clip(myGain + 1, minGain, maxGain );
                setManualExposure(myExposure, myGain);
            } else if (thisGainDn && !lastGainDn) {
                myGain = Range.clip(myGain - 1, minGain, maxGain );
                setManualExposure(myExposure, myGain);
            }

            lastExpUp = thisExpUp;
            lastExpDn = thisExpDn;
            lastGainUp = thisGainUp;
            lastGainDn = thisGainDn;

            sleep(20);
        }
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the WEBCAM vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
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
        myTfodProcessor.setMinResultConfidence((float) 0.10); //0.40 //.10 //lower for when at court

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
        visionPortal = myVisionPortalBuilder.build();

    }

    /*
        Manually set the camera gain and exposure.
        Can only be called AFTER calling initAprilTag();
        Returns true if controls are set.
     */
    private boolean    setManualExposure(int exposureMS, int gain) {
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

    /*
        Read this camera's minimum and maximum Exposure and Gain settings.
        Can only be called AFTER calling initAprilTag();
     */
    private void getCameraSetting() {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return;
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

        // Get camera control values unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            minExposure = (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposure = (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            minGain = gainControl.getMinGain();
            maxGain = gainControl.getMaxGain();
        }
    }
}
