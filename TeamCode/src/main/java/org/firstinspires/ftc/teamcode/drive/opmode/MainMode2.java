package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//import org.firstinspires.ftc.teamcode.drive.additions.*;

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
public class MainMode2 extends LinearOpMode{
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
        TURNING_TO_TARGET,
        DRIVING_STRAIGHT_TO_TARGET,
        AT_TARGET,
        FINDING_HOME,
        DRIVING_HOME
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
            "/sdcard/FIRST/tflitemodels/TennisBallModel1.tflite";
    //labels of model object
    private static final String[] LABELS = {
            "a"
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    //private AprilTagProcessor //TODO: add april tag capability
    private ArrayList<double[]> tfodPositions = new ArrayList<>();
    private int detectionIndex = -1;

    private int ballsCollected = 0;

    private int loopTries = 0;

    public Pose2d startPose;
    Pose2d poseEstimate;

    //variable for the robot
    Robot robot;


    @Override
    public void runOpMode() throws InterruptedException {
        //initializes the robot
        robot = new Robot(hardwareMap);

        initTfod();

        setStartPose();

        waitForStart();

        if (isStopRequested())
            return;


        while (opModeIsActive() && !isStopRequested()) {
            robot.update();

            poseEstimate = robot.getPoseEstimate();

            //allows gamepad to switch modes or states
            universalGamePadCommands();

            updateTelemetry();

            //difference happenings for DRIVER_CONTROL vs AUTO_CONTROL
            switch (currentMode) {
                case AUTO_CONTROL:
                    if (ballsCollected > 2) {
                        currentState = State.HOME;
                    }

                    switch (currentState) {
                        case LOCATE:

                            break;

                        case COLLECT:

                            break;

                        case IDLE:
                            robot.intakeOff();
                            robot.cancelFollowing();

                            break;

                        case HOME:

                            break;
                    }

                    break;

                case DRIVER_CONTROL:
                    //driver-control specific commands
                    driverControlGamepadCommands();

                    break;
            }

            //place pose in storage
            PoseStorage.currentPose = robot.getPoseEstimate();
        }
    }


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
        tfod.setMinResultConfidence(0.65f);

    }

    //sets the startPose based on startPoseEnum
    private void setStartPose() {
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
    }

    //allows the gamepad to change mode and state
    private void universalGamePadCommands() {
        if (gamepad1.x) {
            robot.intakeOff();
            robot.cancelFollowing();
            currentMode = Mode.DRIVER_CONTROL;

        } else if (gamepad1.y) {
            robot.intakeOff();
            currentMode = Mode.AUTO_CONTROL;

        } else if (gamepad1.a) {
            currentState = State.LOCATE;

        } else if (gamepad1.b) {
            robot.intakeOff();
            currentState = State.IDLE;

        }
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

        //tfod
        tfodTelemetry();

        telemetry.update();
    }

    private void tfodTelemetry() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        telemetry.addData("# objects detected", currentRecognitions.size());

        //display info on each recognition
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% conf.)",
                    recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f, %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f",
                    recognition.getWidth(), recognition.getHeight());
        }
    }

    //used for driver control-specific commands
    private void driverControlGamepadCommands() {
        robot.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

        if (gamepad1.left_bumper) {
            robot.intakeOff();

        } else if (gamepad1.right_bumper) {
            robot.intakeOn();

        } else if (gamepad1.left_trigger > 0.1) {
            robot.intakeVariablePower(-gamepad1.left_trigger);

        } else if (gamepad1.right_trigger > 0.1) {
            robot.intakeVariablePower(gamepad1.right_trigger);

        }
    }

    private void tfodFindClosest() {}

}