package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.firstinspires.ftc.teamcode.drive.additions.*;

import java.util.ArrayList;
import java.util.List;

/**
 * //GAMEPAD CONTROLS
 *
 * Universal
 * x    switch mode to driver control, intake off, cancel following
 * y    switch mode to auto control, intake off
 * a    switch state to locate
 * b    switch state to idle
 *
 * Driver control
 * left bumper      intake off
 * right bumper     intake on
 * left trigger     intake reverse with variable speed
 * right trigger    intake collect with variable speed
 */

@TeleOp(group= "APushBot")
//@Disabled
public class MainMode3 extends LinearOpMode{
    //2 modes for if driver is controlling or auto is
    enum Mode {
        DRIVER_CONTROL,
        AUTO_CONTROL
    }
    Mode currentMode = Mode.DRIVER_CONTROL;

    //states for which period of auto control is occuring
    enum State {
        IDLE,
        LOCATE,
        COLLECT,
        HOME
    }
    State currentState = State.IDLE;

    //to keep track of the specific current movement being performed
    enum Movement {
        IDLE,
        LOCATING_BY_ROTATING,
        LOCATING_BY_DRIVING,
        ROTATING_TO_TARGET,
        DRIVING_STRAIGHT_TO_TARGET,
        COLLECTING_TARGET,
        FINDING_HOME,
        DRIVING_HOME,
        DRIVER_IN_CONTROL
    }
    Movement currentMovement = Movement.IDLE;

    //to determine where the robot is located
    enum StartPoseEnum {
        ONE,
        TWO,
        THREE,
        FOUR
    }
    StartPoseEnum startPoseEnum = StartPoseEnum.ONE;

    //tfod file uploaded using ftc interface
    private static final String TFOD_MODEL_FILE =
            "/sdcard/FIRST/tflitemodels/TennisBallModel2.tflite";
    //labels of model object
    private static final String[] LABELS = {
            "a"
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    //private AprilTagProcessor //TODO: add april tag capability
    //for the current streamed recognitions
    private List<Recognition> currentRecognitions;
    private int currentDetectionIndex = -1;
    //for the recognitions we are heading to
    private List<Recognition> savedRecognitions;
    private int savedDetectionIndex;
    private double savedHeadingError;
    private double savedDistance;
    private double savedX, savedY;
    private int locateRotations = 0;
    private int collectRotations = 0;
    private int collectStraights = 0;

    private int ballsCollected = 0;

    public Pose2d startPose;
    Pose2d poseEstimate;

    Timer timer1 = new Timer();
    Timer timer2 = new Timer();
    Timer timer3 = new Timer();



    @Override
    public void runOpMode() throws InterruptedException {
        //initialize robot
        Robot robot = new Robot(hardwareMap);

        initTfod();

        //sets the startPose based on startPoseEnum
        switch (startPoseEnum) {
            case ONE:
                startPose = new Pose2d(0,0,0); //TODO: Change values

                break;

            case TWO:
                startPose = new Pose2d(100,0,0);

                break;

            case THREE:
                break;

            case FOUR:
                break;

            default:
                startPose = new Pose2d(0,0,0); //TODO: Change values
        }

        robot.setPoseEstimate(startPose);
        PoseStorage.currentPose = robot.getPoseEstimate();

        waitForStart();

        if (isStopRequested())
            return;


        while (opModeIsActive() && !isStopRequested()) {
            robot.update();

            poseEstimate = robot.getPoseEstimate();

            //allows the gamepad to change mode and state
            if (gamepad1.x) {
                robot.intakeOff();
                robot.cancelFollowing();
                currentMode = Mode.DRIVER_CONTROL;
                currentMovement = Movement.DRIVER_IN_CONTROL;

            } else if (gamepad1.y) {
                robot.intakeOff();
                currentMode = Mode.AUTO_CONTROL;
                currentMovement = Movement.IDLE;

            } else if (gamepad1.a) {
                currentState = State.LOCATE;

            } else if (gamepad1.b) {
                robot.intakeOff();
                currentState = State.IDLE;

            }

            updateTelemetry();

            //tfod telemetry and also updates the recognitions and
            //the one with the highest confidence
            updateTfod();

            //difference happenings for DRIVER_CONTROL vs AUTO_CONTROL
            switch (currentMode) {
                case AUTO_CONTROL:
                    if (ballsCollected > 2) {
                        currentState = State.HOME;
                    }

                    switch (currentState) {
                        case LOCATE:
                            if (currentDetectionIndex > -1) {
                                savedRecognitions = currentRecognitions;
                                savedDetectionIndex = currentDetectionIndex;
                                savedHeadingError = savedRecognitions
                                        .get(savedDetectionIndex).estimateAngleToObject(AngleUnit.DEGREES);

                                savedX = (savedRecognitions.get(savedDetectionIndex).getLeft()
                                        + savedRecognitions.get(savedDetectionIndex).getRight()) / 2;
                                savedY = (savedRecognitions.get(savedDetectionIndex).getTop()
                                        + savedRecognitions.get(savedDetectionIndex).getBottom()) / 2;

                                savedDistance = 13100000 - 214233*savedY + 1461*savedY*savedY
                                        - 5.31*Math.pow(savedY, 3) + 0.0109*Math.pow(savedY, 4)
                                        - 0.0000118*Math.pow(savedY, 5)
                                        + 0.00000000536*Math.pow(savedY, 6);
                                //1.31E+07 + -214233x + 1461x^2 + -5.31x^3 + 0.0109x^4 + -1.18E-05x^5 + 5.36E-09x^6

                                currentState = State.COLLECT;

                            } else if (timer1.isOver() && !robot.isSimpleTurning()){
                                if (locateRotations > 35) {
                                    driveToNewPosition(); //TODO: Implement
                                    currentMovement = Movement.LOCATING_BY_DRIVING;

                                } else {
                                    robot.simpleTurn(10, 0.8);
                                    timer1.start(0.5);
                                    currentMovement = Movement.LOCATING_BY_ROTATING;

                                }

                            }


                        case COLLECT:
                            if (collectRotations == 0 ) {
                                robot.simpleTurn(savedHeadingError, 0.3);
                                currentMovement = Movement.ROTATING_TO_TARGET;
                                collectRotations++;

                            } else if (robot.isSimpleTurning() || robot.isSimpleStraight()) {

                            } else if (collectStraights == 0) {
                                //min viewing distance 57 inches, add 24 in to get 81
                                robot.simpleStraight(Math.max(0, savedDistance - 81), 0.8);
                                collectStraights++;
                                currentMovement = Movement.DRIVING_STRAIGHT_TO_TARGET;

                            } else if (collectStraights == 1) {
                                robot.simpleStraight(90, 1);
                                currentMovement = Movement.COLLECTING_TARGET;

                            } else {
                                currentState = State.LOCATE;

                            }

                            break;

                        case IDLE:
                            robot.intakeOff();
                            robot.cancelFollowing();

                            break;

                        case HOME:
                            //TODO: Implement

                            break;
                    }

                    break;

                case DRIVER_CONTROL:
                    //used for driver control-specific commands
                    robot.setWeightedDrivePower(
                            new Pose2d(
                                    (gamepad1.dpad_up ? 1 : (gamepad1.dpad_down ? -1 : -gamepad1.left_stick_y)),
                                    (gamepad1.dpad_left ? 1 : (gamepad1.dpad_right ? -1 : -gamepad1.left_stick_x)),
                                    -gamepad1.right_stick_x
                            )
                    );

                    //if triggers are 0 turn off intake if it is not continuous
                    if (gamepad1.left_trigger < 0.1 && gamepad1.right_trigger < 0.1 && !robot.isIntakeContinuous()) {
                        robot.intakeVariablePower(0);
                    }

                    if (gamepad1.left_bumper) {
                        robot.intakeOff();

                    } else if (gamepad1.right_bumper) {
                        robot.intakeOn();

                    } else if (gamepad1.left_trigger > 0.1) {
                        robot.intakeVariablePower(-gamepad1.left_trigger);

                    } else if (gamepad1.right_trigger > 0.1) {
                        robot.intakeVariablePower(gamepad1.right_trigger);

                    }

                    break;
            }

            //robot.checkMotorPositions(); //TODO

            //place pose in storage
            PoseStorage.currentPose = robot.getPoseEstimate();
        }
    }

    //initialize the object detection
    private void initTfod() {
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
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.setAutoStopLiveView(false);
        builder.addProcessor(tfod);

        visionPortal = builder.build();

        //the confidence interval for objects to be recognized
        tfod.setMinResultConfidence(0.10f);
    }

    //regulates the telemetry on the screen
    private void updateTelemetry() {
        //modes, states, and movement
        telemetry.addData("mode", currentMode);
        telemetry.addData("state", currentState);
        telemetry.addData("movement", currentMovement);

        //pose
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("", "---------------"); //15 dashes

        telemetry.update();
    }

    //Add the telemetry for the object detection and the highest confidence index
    private void updateTfod() {
        currentRecognitions = tfod.getRecognitions();

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

    private void driveToNewPosition() {

    }
}