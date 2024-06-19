package org.firstinspires.ftc.teamcode.updatedDrive2.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "TFODpropAprilTag (Blocks to Java)")
public class autoTfodAndAprilTagAndDrive extends LinearOpMode {

    private DcMotor left_driveAsDcMotor;
    private DcMotor right_driveAsDcMotor;

    AprilTagProcessor myAprilTagProcessor;
    List<Recognition> myTfodRecognitions;
    TfodProcessor myTfodProcessor;
    ElapsedTime myElapsedTime;
    VisionPortal myVisionPortal;
    boolean USE_WEBCAM;
    AprilTagDetection desiredTag;
    double COUNTS_PER_INCH;
    int DESIRED_TAG_ID;
    int DESIRED_DISTANCE;
    double SPEED_GAIN;
    double TURN_GAIN;
    double MAX_AUTO_SPEED;
    double MAX_AUTO_TURN;

    /**
     * This is a complete CENTERSTAGE autonomous program for the Blue rear starting location.
     * It detects a Team Prop on the spike mark, places the purple pixel on that mark and then parks backstage.
     */
    @Override
    public void runOpMode() {
        int COUNTS_PER_MOTOR_REV;
        int DRIVE_GEAR_REDUCTION;
        int WHEEL_DIAMETER_INCHES;
        double DRIVE_SPEED;
        double TURN_SPEED;
        String location;

        left_driveAsDcMotor = hardwareMap.get(DcMotor.class, "left_driveAsDcMotor");
        right_driveAsDcMotor = hardwareMap.get(DcMotor.class, "right_driveAsDcMotor");

        // Put initialization blocks here.
        // This is a complete CENTERSTAGE autonomous program for the Blue rear starting location.
        // It detects a Team Prop on the spike mark and places the purple pixel on that mark.
        // It then turns to face the backdrop and uses April Tag controlled driving to drive to the backdrop area corresponding to the spike mark.
        // Change the left motor direction so positive motor power (or motor target positions) will move robot forward.
        left_driveAsDcMotor.setDirection(DcMotor.Direction.REVERSE);
        // Add an ElapsedTime variable and set it to a new Elapsed Time object, we'll use it to control timeouts in the encoderDrive funtion.
        myElapsedTime = new ElapsedTime();
        COUNTS_PER_MOTOR_REV = 1100;
        DRIVE_GEAR_REDUCTION = 1;
        WHEEL_DIAMETER_INCHES = 4;
        // Set COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV - DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
        DRIVE_SPEED = 0.5;
        TURN_SPEED = 0.3;
        // Initialize encoders
        left_driveAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_driveAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_driveAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_driveAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Initialize variables for April Tag driving
        DESIRED_DISTANCE = 10;
        // Setting SPEED_GAIN to 0.03 so the program is a bit better about reaching the DESIRED_DISTANCE.
        SPEED_GAIN = 0.03;
        TURN_GAIN = 0.01;
        MAX_AUTO_SPEED = 0.5;
        MAX_AUTO_TURN = 0.25;
        DESIRED_TAG_ID = -1;
        USE_WEBCAM = true;
        desiredTag = null;
        initVisionPortal();
        // Display a message that initialization is complete.
        telemetry.addLine("Initialization is complete, press the Start button to run the program.");
        telemetry.update();
        waitForStart();
        // Now wait for the Start button to be pressed.
        if (opModeIsActive()) {
            // Put run blocks here.
            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
            telemetry.addData("STEP", "1 - detect prop");
            telemetry.update();
            if (detectProp()) {
                // If a prop was detected, determine whether it was the left, center or right spike mark
                // We're using 640x480 so the middle detection should be under 400
                telemetry.addData("TensorFlow", "Team Prop Found");
                if (((Recognition) JavaUtil.inListGet(myTfodRecognitions, JavaUtil.AtMode.FROM_START, 0, false)).getLeft() < 10) {
                    location = "left";
                } else if (((Recognition) JavaUtil.inListGet(myTfodRecognitions, JavaUtil.AtMode.FROM_START, 0, false)).getLeft() < 400) {
                    location = "center";
                } else {
                    location = "right";
                }
            } else {
                // If a prop was NOT detected, just assume it's in the center.
                telemetry.addData("TensorFlow", "Team Prop not found");
                location = "center";
            }
            telemetry.addData("Location", location);
            telemetry.update();
            sleep(1000);
            if (location.equals("center")) {
                telemetry.addData("STEP", "2 - drive to center spike mark");
                telemetry.update();
                encoderDrive(DRIVE_SPEED, 33, 33, 3);
                telemetry.addData("STEP", "3 - back up and turn to face the backdrop");
                telemetry.update();
                encoderDrive(DRIVE_SPEED, -18, -3, 3);
                // The above turn doesn't need to be accurate, just enough so the webcam can see all three april tags on the backdrop
            } else if (location.equals("left")) {
                // to be completed later
                telemetry.addData("STEP", "left spike mark code not implemented");
                telemetry.update();
                sleep(2000);
            } else {
                // to be completed later
                telemetry.addData("STEP", "right spike mark code not implemented");
                telemetry.update();
                sleep(2000);
            }
            // Let's assume the code above always end with the robot turned to face the backdrop, now drive towards the backdrop
            // Now set manual webcam exposure to improve April Tag detection
            if (USE_WEBCAM) {
                // Use short exposure time and high gain to reduce motion blur
                setManualExposure(6, 250);
            }
            if (location.equals("center")) {
                DESIRED_TAG_ID = 2;
            } else if (location.equals("left")) {
                DESIRED_TAG_ID = 1;
            } else {
                // just assume location = right
                DESIRED_TAG_ID = 3;
            }
            telemetry.addData("STEP", "4 -April Tag drive to backdrop");
            telemetry.update();
            aprilTagDrive();
            // In testing the aprilTagDrive program with DESIRED_DISTANCE=10 the webcam ends up 11" in front of the target April Tag.
            // That's about a close as the robot can get as the April Tag is almost out of the webcam view when the robot is that close.
            telemetry.addData("STEP", "5 - encoder drive up to backdrop");
            telemetry.update();
            // try 0.2 speed, don't want to hit backdrop hard, we add extra right wheel distance to try and line up on the backdrop
            encoderDrive(0.2, 7, 9, 4);
            telemetry.addLine("Autonomous Complete");
            telemetry.update();
            sleep(2000);
        }
    }

    /**
     * Describe this function...
     */
    private void aprilTagDrive() {
        int rangeError;
        boolean targetFound;
        double drive;
        double turn;
        List<AprilTagDetection> myAprilTagDetections;
        AprilTagDetection detection;
        double headingError;

        // Drive towards the April Tag given by the DESIRED_TAG_ID variable
        // This function is mostly made up of the main loop in from the RobotAutoDriveAprilTagTank_Blocks program.
        rangeError = 10;
        // initialize rangeError so we can start the loop. We'll exit the loop when the range is "close enough", in this case 2 inches.
        while (rangeError > 2) {
            // Put loop blocks here.
            targetFound = false;
            desiredTag = null;
            // Step through the list of detected tags and look for a matching tag
            // Get a list containing the latest detections, which may be stale.
            myAprilTagDetections = myAprilTagProcessor.getDetections();
            for (AprilTagDetection detection_item : myAprilTagDetections) {
                detection = detection_item;
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    // Check to see if we want to track towards this tag.
                    if (DESIRED_TAG_ID == detection.id) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        // don't look any further.
                        break;
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                }
            }
            // Now start driving
            if (targetFound) {
                telemetry.addData("Found", " Tag ID " + desiredTag.id + " (" + desiredTag.metadata.name + ")");
                telemetry.addData("Range", JavaUtil.formatNumber(desiredTag.ftcPose.range, 5, 1) + " inches");
                telemetry.addData("Bearing", JavaUtil.formatNumber(desiredTag.ftcPose.bearing, 3, 0) + " degrees");
                // Determine heading and range error so we can use them to control the robot automatically.
                rangeError = (int) (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                headingError = desiredTag.ftcPose.bearing;
                // Use the speed and turn "gains" to calculate how we want the robot to move.  Clip it to the maximum
                drive = Math.min(Math.max(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED), MAX_AUTO_SPEED);
                turn = Math.min(Math.max(headingError * TURN_GAIN, -MAX_AUTO_TURN), MAX_AUTO_TURN);
                telemetry.addData("Auto", "Drive " + JavaUtil.formatNumber(drive, 5, 2) + ", Turn " + JavaUtil.formatNumber(turn, 5, 2));
            } else {
                drive = 0;
                turn = 0;
            }
            telemetry.update();
            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, turn);
            sleep(10);
        }
        // ensure robot stops when loop exits
        drive = 0;
        turn = 0;
        moveRobot(drive, turn);
    }

    /**
     * Describe this function...
     */
    private void setManualExposure(int exposureMS, int gain) {
        ExposureControl myExposureControl;
        GainControl myGainControl;

        // Manually set the camera gain and exposure.
        // This can only be called AFTER calling initAprilTag(), and only works for Webcams;
        // This function copied from the RobotAutoDriveAprilTagTank_Blocks program.
        if (myVisionPortal.equals(null)) {
            return;
        }
        // Wait for the camera to be open, then use the controls
        // Make sure camera is streaming before we try to set the exposure controls
        // Get the state of the camera.
        if (!myVisionPortal.getCameraState().equals(VisionPortal.CameraState.STREAMING)) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            // Get the state of the camera.
            while (opModeIsActive() && !myVisionPortal.getCameraState().equals(VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
        // Set camera controls unless we are stopping.
        if (opModeIsActive()) {
            // Get the ExposureControl object, to allow adjusting the camera's exposure.
            myExposureControl = myVisionPortal.getCameraControl(ExposureControl.class);
            // Get the current exposure mode value.
            if (!myExposureControl.getMode().equals(ExposureControl.Mode.Manual)) {
                // Set the exposure mode value.
                myExposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            // Set the exposure.
            myExposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            // Get the GainControl object, to allow adjusting the camera's gain.
            myGainControl = myVisionPortal.getCameraControl(GainControl.class);
            // Set the gain. Requires Exposure Control Mode to be MANUAL.
            myGainControl.setGain(gain);
            sleep(20);
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
    }

    /**
     * Describe this function...
     */
    private void moveRobot(double x, double yaw) {
        double leftPower;
        double rightPower;
        double max;

        // Move robot according to desired axes motions
        // This function copied from the RobotAutoDriveAprilTagTank_Blocks program.
        // Positive X is forward
        // Positive Yaw is counter-clockwise
        // Calculate left and right wheel powers.
        leftPower = x - yaw;
        rightPower = x + yaw;
        // Calculate left and right wheel powers.
        max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftPower), Math.abs(rightPower)));
        if (max > 1) {
            // Normalize wheel powers to be less than 1.0
            leftPower = leftPower / max;
            rightPower = rightPower / max;
        }
        // Send powers to the wheels.
        left_driveAsDcMotor.setPower(leftPower);
        right_driveAsDcMotor.setPower(rightPower);
    }

    /**
     * Describe this function...
     */
    private void initVisionPortal() {
        VisionPortal.Builder myVisionPortalBuilder;

        // First create April Tag and TensorFlow processors, then init the vision portal.
        initTfod();
        initAprilTag();
        // Create the vision portal by using a builder.
        // Create a VisionPortal.Builder object so you can specify attributes about the cameras.
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            // Set the camera to the specified webcam name.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Add the AprilTag processor.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        // Add the TensorFlow processor.
        myVisionPortalBuilder.addProcessor(myTfodProcessor);
        // Build the VisionPortal object and assign it to a variable.
        myVisionPortal = myVisionPortalBuilder.build();
    }

    /**
     * Describe this function...
     */
    private void initAprilTag() {
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
        myAprilTagProcessor.setDecimation(2);
    }

    /**
     *      *  Function to perform a relative move, based on encoder counts.
     *      *  Encoders are not reset as the move is based on the current position.
     *      *  Move will stop if any of three conditions occur:
     *      *  1) Move gets to the desired position
     *      *  2) Move runs out of time
     *      *  3) Driver stops the OpMode running.
     *
     * Input parameters:
     * speed - power level for the motors
     * leftInches - the distance we want the left wheel to drive
     * rightInches - the distance we wan the right wheel to drive
     * timoutS - a timeout in seconds in case the robot is blocked before reaching the target inches.
     */
    private void encoderDrive(double speed, int leftInches, int rightInches, int timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Function to perform a relative move, based on encoder counts.
        // Encoders are not reset as the move is based on the current position.
        // Move will stop if any of three conditions occur:
        // 1) Move gets to the desired position
        // 2) Move runs out of time
        // 3) Driver stops the OpMode running.
        if (opModeIsActive()) {
            // Ensure that the OpMode is still active
            // Determine new target position, and pass to motor controller
            newLeftTarget = (int) (left_driveAsDcMotor.getCurrentPosition() + leftInches * COUNTS_PER_INCH);
            newRightTarget = (int) (right_driveAsDcMotor.getCurrentPosition() + rightInches * COUNTS_PER_INCH);
            left_driveAsDcMotor.setTargetPosition(newLeftTarget);
            right_driveAsDcMotor.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            left_driveAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_driveAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion
            myElapsedTime.reset();
            left_driveAsDcMotor.setPower(Math.abs(speed));
            right_driveAsDcMotor.setPower(Math.abs(speed));
            while (opModeIsActive() && myElapsedTime.seconds() < timeoutS && (left_driveAsDcMotor.isBusy() || right_driveAsDcMotor.isBusy())) {
                // keep looping while we are still active, and there is time left, and at least one  motor is running
            }
            // stop all motion
            left_driveAsDcMotor.setPower(0);
            right_driveAsDcMotor.setPower(0);
            // Turn off RUN_TO_POSITION
            left_driveAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_driveAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // optional: pause after each move, in a real autonomous program you probably don't want unnecessary delays.
        }
    }

    /**
     * Initialize TensorFlow Object Detection.
     * The team prop names are set and the setMinResultConfidencs is set to 0.70 instead of 0.75.
     */
    private void initTfod() {
        TfodProcessor.Builder myTfodProcessorBuilder;

        // First, create a TfodProcessor.
        // Create a new TfodProcessor.Builder object.
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        // Set the name of the file where the model can be found.
        myTfodProcessorBuilder.setModelFileName("redBlueDuploFar.tflite");
        // Set the full ordered list of labels the model is trained to recognize.
        myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith("blueDuplo", "redDuplo"));
        // Build the TensorFlow Object Detection processor and assign it to a variable.
        myTfodProcessor = myTfodProcessorBuilder.build();
        // Set the minimum confidence at which to keep recognitions.
        myTfodProcessor.setMinResultConfidence((float) 0.7);
    }

    /**
     * Describe this function...
     */
    private boolean detectProp() {
        boolean propFound;

        // This function returns true if a Team Prop was detected and false if it was not.
        // Call getRecognitions to get a list of recognitions(detected objects), save in variable myTfodRecognitions.
        // Get a list containing the latest recognitions, which may be stale.
        myTfodRecognitions = myTfodProcessor.getRecognitions();
        myElapsedTime.reset();
        // We need to go into a Repeat loop that keeps running until we timeout or we find something.
        // Note: It can take TensorFlow a second or two to find an object, and we expect we will NOT find an object fairly often.
        // myTfodRecognitions is a list, we can check the Length of this list and if it is zero we haven't found one yet.
        while (opModeIsActive() && myElapsedTime.seconds() < 3 && JavaUtil.listLength(myTfodRecognitions) == 0) {
            // we call sleep here so that TensorFlow might have some more time to detect something, no need to check too often
            sleep(250);
            // Get a list containing the latest recognitions, which may be stale.
            myTfodRecognitions = myTfodProcessor.getRecognitions();
        }
        // myTfodRecognitions is a list, we could scan the list looking for a Team Prop label or simply check if anything is detected.
        if (JavaUtil.listLength(myTfodRecognitions) > 0) {
            propFound = true;
        } else {
            propFound = false;
        }
        return propFound;
    }
}