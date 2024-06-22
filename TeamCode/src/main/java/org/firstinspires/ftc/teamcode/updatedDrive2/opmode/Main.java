package org.firstinspires.ftc.teamcode.updatedDrive2.opmode;

import static org.firstinspires.ftc.teamcode.updatedDrive2.constants.Constants.RESTING_INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.updatedDrive2.main.TfodControls.savedDetectionIndex;
import static org.firstinspires.ftc.teamcode.updatedDrive2.main.TfodControls.savedDistance;
import static org.firstinspires.ftc.teamcode.updatedDrive2.main.TfodControls.savedHeadingError;
import static org.firstinspires.ftc.teamcode.updatedDrive2.main.TfodControls.savedRecognitions;
import static org.firstinspires.ftc.teamcode.updatedDrive2.main.TfodControls.savedX;
import static org.firstinspires.ftc.teamcode.updatedDrive2.main.TfodControls.savedY;
import static org.firstinspires.ftc.teamcode.updatedDrive2.storage.PoseStorage.poseEstimate;
import static org.firstinspires.ftc.teamcode.updatedDrive2.storage.Positions.Mode;
import static org.firstinspires.ftc.teamcode.updatedDrive2.storage.Positions.Movement;
import static org.firstinspires.ftc.teamcode.updatedDrive2.storage.Positions.StartPoseEnumerated;
import static org.firstinspires.ftc.teamcode.updatedDrive2.storage.Positions.State;
import static org.firstinspires.ftc.teamcode.updatedDrive2.storage.Positions.currentMode;
import static org.firstinspires.ftc.teamcode.updatedDrive2.storage.Positions.currentMovement;
import static org.firstinspires.ftc.teamcode.updatedDrive2.storage.Positions.currentState;
import static org.firstinspires.ftc.teamcode.updatedDrive2.storage.Positions.startPoseEnumerated;
import static org.firstinspires.ftc.teamcode.updatedDrive2.objects.Drivetrain.angleSinceLastRotation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.math.BigDecimal;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.updatedDrive2.main.Robot;
import org.firstinspires.ftc.teamcode.updatedDrive2.storage.PoseStorage;
import org.firstinspires.ftc.teamcode.updatedDrive2.storage.Positions;

import java.util.Timer;


/**
 * The main opmode for the robot
 *
 *
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


//TODO: Add distance sensor to prevent driving into solid objects and add virtual map for path planning


@TeleOp(name="updatedDrive2Main", group = "APushBot")
//@Disabled
public class Main extends LinearOpMode {
    private int locateRotations = 0;
    private int collectRotations = 0;
    private int collectStraights = 0;

    private boolean firstAuto = true; //have we been in auto yet or have we just picked up a ball?
    ElapsedTime timer = new ElapsedTime();

    private int ballsCollected = 0;

    public Pose2d startPose;

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, gamepad1);

        currentMode = Mode.DRIVER_CONTROL;
        currentState = State.IDLE;
        currentMovement = Movement.DRIVER_IN_CONTROL;

        startPoseEnumerated = StartPoseEnumerated.COURT_FOUR;
        startPose = Positions.getStartPose();

        robot.drivetrain.setPoseEstimate(startPose);
        PoseStorage.currentPose = robot.drivetrain.getPoseEstimate();

        //set previous trajectory as the starting point
        robot.drivetrain.previousTrajectory = robot.drivetrain.trajectoryBuilder(startPose).forward(0.1).build();


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            poseEstimate = robot.drivetrain.getPoseEstimate();

            //checks and does actions based on the universal gamepad controls
            //which occur no matter what
            robot.gamepadControls.universalControls();

            //updates the drivetrain, distance sensor, tfod, AprilTags, etc
            //Includes telemetry update
            robot.update();

            switch (currentMode) {
                case AUTO_CONTROL:

                    /*
                    if (currentState != State.IDLE && currentState != State.DROPPING_OFF && ballsCollected > 9) {
                        //if the robot has collected enough targets, head home if not idle or dropping off
                        currentState = State.HEADING_HOME;

                    } else {*/
                        switch (currentState) {
                            case IDLE:
                                //stop movement and intake
                                robot.drivetrain.setMotorPowers(0.0, 0.0, 0.0, 0.0);
                                robot.intake.turnIntakeOff();

                                currentMovement = Movement.IDLE;

                                break;

                            case LOCATING:
                                //locates tennis balls
                                robot.intake.intakeVariablePower(RESTING_INTAKE_POWER);

                                //if there is a detection
                                if (robot.tfodControls.currentRecognitions.size() > 0) {
                                    //save the recognition data for the highest confidence recognition
                                    robot.tfodControls.saveRecognitionData();

                                    currentState = State.COLLECTING;

                                    //if the distance seems off
                                    if (savedDistance < 0 || savedDistance > 300) {
                                        robot.drivetrain.simpleRotate(360);
                                        currentMode = Mode.FIRST_AUTO_CONTROL;
                                        currentState = Positions.State.LOCATING;
                                    }

                                } else if (!robot.drivetrain.isSimpleRotating()) {
                                    //if there is no detection
                                    if (locateRotations < 272 /*72*/) {
                                        //rotate to find target
                                        robot.drivetrain.simpleRotate(5.0); //rotate 10 degrees counterclockwise
                                        locateRotations++;

                                        currentMovement = Movement.LOCATING_BY_ROTATING;

                                    } else if (!robot.drivetrain.isSimpleStraight()){
                                        //drive to a new location asynchronously
                                        findNewLocation();
                                        driveToNewLocation();

                                        //once done: locateRotations = 0;

                                        currentMovement = Movement.LOCATING_BY_DRIVING;

                                    }

                                }

                                break;

                            case COLLECTING:
                                //collects the detected tennis ball
                                if (!robot.drivetrain.isSimpleRotating() || !robot.drivetrain.isSimpleStraight()) {
                                    if (collectRotations == 0) {
                                        //needs to rotate to target
                                        robot.drivetrain.simpleRotate(savedHeadingError);
                                        collectRotations++;

                                        currentMovement = Movement.ROTATING_TO_TARGET;

                                    } else if (collectStraights == 0) {
                                        //needs to collect target
                                        robot.intake.turnIntakeOn();

                                        robot.drivetrain.simpleStraight(savedDistance + 3);
                                        collectStraights++;

                                        currentMovement = Movement.DRIVING_STRAIGHT_TO_TARGET;

                                    } /*else if (robot.drivetrain.isSimpleStraightJustEnd()) {
                                    currentMovement = Movement.COLLECTING_TARGET;

                                    } */else {
                                        //collected target
                                        ballsCollected++;

                                        collectRotations = 0;
                                        collectStraights = 0;

                                        currentState = State.LOCATING;
                                        firstAuto = true;
                                        currentMode = Mode.FIRST_AUTO_CONTROL;

                                    }

                                }

                                break;

                            case HEADING_HOME:
                                //finds apriltag and then when found go to it
                                robot.intake.intakeVariablePower(RESTING_INTAKE_POWER);

                                currentMovement = Movement.FINDING_HOME;

                                break;

                            case DROPPING_OFF:
                                //dropping off the tennis balls when already at home
                                robot.intake.turnIntakeOff();

                                currentMovement = Movement.STATIONARY_FOR_DROPOFF;

                        }

                    //}

                    break;

                case DRIVER_CONTROL:
                    //gamepad controls that only occur if the mode is driver control
                    robot.gamepadControls.driverControlControls();

                    currentMovement = Movement.DRIVER_IN_CONTROL;

                    break;

                case FIRST_AUTO_CONTROL:

                    if (firstAuto) {
                        timer.reset();
                        firstAuto = false;

                    } else if (timer.seconds() > 2.0) {
                        firstAuto = true;
                        currentMode = Mode.AUTO_CONTROL;
                        currentState = State.LOCATING;

                    }

            }

            //updates only the telemetry
            robot.telemetryControls.update();


        }

    }


    /** FUNCTIONS ***/




    //TODO: finish these functions
    //find new location using virtual map
    private void findNewLocation() {

    }

    //drives to new location avoiding objects
    private void driveToNewLocation() {

    }

}
