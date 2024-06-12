package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.Robot;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * This opmode demonstrates how one can augment driver control by following Road Runner arbitrary
 * Road Runner trajectories at any time during teleop. This really isn't recommended at all. This is
 * not what Trajectories are meant for. A path follower is more suited for this scenario. This
 * sample primarily serves as a demo showcasing Road Runner's capabilities.
 * <p>
 * This bot starts in driver controlled mode by default. The player is able to drive the bot around
 * like any teleop opmode. However, if one of the select buttons are pressed, the bot will switch
 * to automatic control and run to specified location on its own.
 * <p>
 * If A is pressed, the bot will generate a splineTo() trajectory on the fly and follow it to
 * targetA (x: 45, y: 45, heading: 90deg).
 * <p>
 * If B is pressed, the bot will generate a lineTo() trajectory on the fly and follow it to
 * targetB (x: -15, y: 25, heading: whatever the heading is when you press B).
 * <p>
 * If Y is pressed, the bot will turn to face 45 degrees, no matter its position on the field.
 * <p>
 * Pressing X will cancel trajectory following and switch control to the driver. The bot will also
 * cede control to the driver once trajectory following is done.
 * <p>
 * The following may be a little off with this method as the trajectory follower and turn
 * function assume the bot starts at rest.
 * <p>
 * This sample utilizes the SampleMecanumDriveCancelable.java class.
 */
@TeleOp(group = "APushBot")
//@Disabled
public class MainMode extends LinearOpMode {
    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector = new Vector2d(45, 45);
    // The heading we want the bot to end on for targetA
    double targetAHeading = Math.toRadians(90);

    // The location we want the bot to automatically go to when we press the B button
    Vector2d targetBVector = new Vector2d(-15, 25);

    // The angle we want to align to when we press Y
    double targetAngle = Math.toRadians(45);

    Trajectory idle, move, collect, goHome, tragectory1, trajectory2; //etc

    Pose2d startPose;

    enum DriveState {
        IDLE,
        MOVE,
        COLLECT,
        HOME
    }

    DriveState currentDriveState = DriveState.IDLE;

    enum ImageState {
        IDLE,
        MOVE,
        COLLECT,
        HOME
    }

    ImageState currentImageState = ImageState.IDLE;

    enum StartPoseEnum {
        FRONT_LEFT,//left entrance towards the parking lot
        BACK_RIGHT
    }

    StartPoseEnum startPoseEnum;

    //private static final String TFOD_MODEL_ASSET = "TennisBallModel2.tflite"; //"RedBlueBox.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/TennisBallModel1.tflite"; //"/sdcard/FIRST/tflitemodels/RedBlueBox.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "a",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private boolean tfodDetected = false;
    private boolean aprilTagDetected = false;

    private ArrayList<double[]> tfodPositions = new ArrayList<double[]>();

    public int ballsCollected = 0;

    private int detectionIndex;

    private boolean moved = false;

    private String movement = "idle";


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize custom cancelable Robot class
        Robot drive = new Robot(hardwareMap);

        //initAprilTag();
        initTfod();

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        startPoseEnum = StartPoseEnum.FRONT_LEFT;

        switch (startPoseEnum) {
            case FRONT_LEFT:

                startPose = new Pose2d(0, 0,0); //TODO: CHANGE VALUES

                break;

            default:

                startPose = new Pose2d(0, 0,0);
        }

        drive.setPoseEstimate(startPose);

        /*
        move = drive.trajectoryBuilder(startPose)
                .forward(100)
                .build();
         */

        PoseStorage.currentPose = drive.getPoseEstimate();

        waitForStart();

        /***
         * BUTTONS
         * a returns to move image state
         * b returns to idle image state
         * x returns to driver control
         *      left bumper turns off intake
         *      right bumper turns on intake
         * y returns to auto
         */

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            if (gamepad1.a) {
                currentImageState = ImageState.MOVE;
            } else if (gamepad1.b) {
                currentImageState = ImageState.IDLE;
            }

            // Print pose to telemetry
            telemetry.addData("mode: ", currentMode);
            telemetry.addData("driveState: ", currentDriveState);
            telemetry.addData("imageState: ", currentImageState);
            telemetry.addData("movement: ", movement);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            detectionIndex = tfodLocationDetection();
            telemetryTfod();

            telemetry.update();



            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.x) {
                        drive.cancelFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    if (ballsCollected > 2) {
                        currentImageState = ImageState.HOME;
                    }

                    //image recognition
                    switch (currentImageState) {
                        case MOVE:
                            drive.intakeOff();

                            if (detectionIndex > -1) {
                                double distance = tfodPositions.get(detectionIndex)[0], angle = tfodPositions.get(detectionIndex)[1];
                                if (distance < 60) {
                                    currentImageState = ImageState.COLLECT;
                                } else {
                                    drive.turn(angle);
                                    Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                                            .forward(distance * 0.7)
                                            .build();

                                    drive.followTrajectory(trajectory);
                                    wait(1000);
                                }

                            } else {
                                drive.turn(Math.toRadians(10));
                            }
                            moved = false;

                        case COLLECT:
                            drive.intakeOn();

                            if (detectionIndex > -1) {
                                double distance = tfodPositions.get(detectionIndex)[0], angle = tfodPositions.get(detectionIndex)[1];
                                if (distance < 24 && !moved) {
                                    Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                                            .forward(30)
                                            .build();

                                    drive.followTrajectory(trajectory);
                                    currentImageState = ImageState.MOVE;

                                    ballsCollected++;
                                    moved = true;
                                } else {
                                    drive.turn(angle);
                                    Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                                            .forward(distance * 0.7)
                                            .build();

                                    drive.followTrajectory(trajectory);
                                    moved = false;
                                }

                            } else {
                                currentImageState = ImageState.MOVE;
                            }

                            break;

                        case HOME:
                            //aprilTagLocationDetection();
                            //telemetryAprilTag();

                            break;

                        default:

                            break;
                    }

                    /*
                    switch (currentDriveState) {
                        case MOVE:
                            //move to a tennis ball
                            drive.intakeOff();

                            //if tennis ball count at 10, run HOME
                            //run webcam vision
                            //for each tennis ball keep locations
                            //go to closest tennis ball by x and y
                            //if near tennis ball run COLLECT
                            //if no tennis balls, rotate
                            //if rotated 2 times, return HOME

                            break;

                        case COLLECT:
                            //collect a tennis ball when at a tennis ball
                            drive.intakeOn();


                            break;

                        case HOME:
                            //head to home spot to drop of the tennis balls
                            drive.intakeOff();

                            break;

                        case IDLE:
                            drive.intakeOff();

                            break;
                    }*/

                    break;

                case DRIVER_CONTROL:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    if (gamepad1.y) {
                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }

                    if (gamepad1.left_bumper) {
                        drive.intakeOff();
                    } else if (gamepad1.right_bumper) {
                        drive.intakeOn();
                    }

                    /*
                    if (gamepad1.a) {
                        // If the A button is pressed on gamepad1, we generate a splineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(targetAVector, targetAHeading)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (gamepad1.b) {
                        // If the B button is pressed on gamepad1, we generate a lineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .lineTo(targetBVector)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (gamepad1.y) {
                        // If Y is pressed, we turn the bot to the specified angle to reach
                        // targetAngle (by default, 45 degrees)

                        //drive.turnAsync(Angle.normDelta(targetAngle - poseEstimate.getHeading()));

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }*/
                    break;
            }
        }
    }

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.60f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }

        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }

    private int tfodLocationDetection()
    {
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        double minDistance = 99999;
        int minDistanceIndex = 0, index = 0;

        // Step through the list of detections and display info for each one.
        for (Recognition recognition : currentRecognitions) {

            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

            //adds the distance and heading of each detection
            tfodPositions.add(new double[]{distance, Math.atan(y/x), x, y});

            //finds closest detection
            if (distance < minDistance) {
                minDistance = distance;
                minDistanceIndex = index;
            }

            index++;
        }

        if (currentRecognitions.size() > 0) {
            return minDistanceIndex;
        } else {
            return -1;
        }

    }
    private void aprilTagLocationDetection()
    {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {

        }


    }

    public void goStraight(double dist) {

    }

}