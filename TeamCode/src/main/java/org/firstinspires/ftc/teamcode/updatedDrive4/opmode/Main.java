package org.firstinspires.ftc.teamcode.updatedDrive4.opmode;

import static org.firstinspires.ftc.teamcode.updatedDrive4.constants.Constants.RESTING_INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.updatedDrive4.constants.Constants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.updatedDrive4.main.CameraControls.savedDistance;
import static org.firstinspires.ftc.teamcode.updatedDrive4.main.CameraControls.savedHeadingError;
import static org.firstinspires.ftc.teamcode.updatedDrive4.objects.REVDistanceSensor.distance;
import static org.firstinspires.ftc.teamcode.updatedDrive4.storage.PoseStorage.poseEstimate;
import static org.firstinspires.ftc.teamcode.updatedDrive4.storage.Positions.Mode;
import static org.firstinspires.ftc.teamcode.updatedDrive4.storage.Positions.Movement;
import static org.firstinspires.ftc.teamcode.updatedDrive4.storage.Positions.StartPoseEnumerated;
import static org.firstinspires.ftc.teamcode.updatedDrive4.storage.Positions.State;
import static org.firstinspires.ftc.teamcode.updatedDrive4.storage.Positions.currentMode;
import static org.firstinspires.ftc.teamcode.updatedDrive4.storage.Positions.currentMovement;
import static org.firstinspires.ftc.teamcode.updatedDrive4.storage.Positions.currentState;
import static org.firstinspires.ftc.teamcode.updatedDrive4.storage.Positions.startPoseEnumerated;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.updatedDrive4.main.Robot;
import org.firstinspires.ftc.teamcode.updatedDrive4.main.TelemetryControls;
import org.firstinspires.ftc.teamcode.updatedDrive4.objects.Drivetrain;
import org.firstinspires.ftc.teamcode.updatedDrive4.storage.PoseStorage;
import org.firstinspires.ftc.teamcode.updatedDrive4.storage.Positions;


/**
 * The main opmode for the robot
 * <p>
 *
 * //GAMEPAD CONTROLS
 * <p>
 * Universal
 * x    switch mode to driver control, intake off, cancel following
 * y    switch mode to auto control, intake off
 * a    switch state to locate
 * b    switch state to idle
 * <p>
 * Driver control
 * left bumper      intake off
 * right bumper     intake on
 * left trigger     intake reverse with variable speed
 * right trigger    intake collect with variable speed
 */


//TODO: Add distance sensor to prevent driving into solid objects and add virtual map for path planning


@TeleOp(name="updatedDrive4Main", group = "APushBot")
//@Disabled
public class Main extends LinearOpMode {
    public static int locateRotations = 0;
    public static int collectRotations = 0;
    public static int collectStraights = 0;

    private boolean firstAuto = true; //have we been in auto yet or have we just picked up a ball?
    private boolean firstDetection = true; //is this the first time detecting objects?
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer1 = new ElapsedTime();

    private int ballsCollected = 0;

    public Pose2d startPose;

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Wait...............");
        telemetry.update();

        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);

        currentMode = Mode.FIRST_AUTO_CONTROL; // Mode.DRIVER_CONTROL;
        currentState = State.LOCATING; //State.IDLE;
        currentMovement = Movement.IDLE; //Movement.DRIVER_IN_CONTROL;

        startPoseEnumerated = StartPoseEnumerated.COURT_FOUR;
        startPose = Positions.getStartPose();

        robot.drivetrain.setPoseEstimate(startPose);
        PoseStorage.currentPose = robot.drivetrain.getPoseEstimate();

        distance = 999;

        //set previous trajectory as the starting point
        robot.drivetrain.currentTrajectory = robot.drivetrain.trajectoryBuilder(startPose).forward(0.000001).build();

        telemetry.addLine("Ready");
        telemetry.update();

        //robot.cameraControls.visionPortal1.stopLiveView();
        robot.cameraControls.visionPortal2.stopLiveView();
        robot.cameraControls.visionPortal2.stopStreaming();

        while (!opModeIsActive() && !isStopRequested()) {
            robot.gamepadControls.universalControls();


        }

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

                    if (currentState == State.IDLE) {
                        //stop movement and intake
                        robot.drivetrain.breakFollowingSmooth();
                        robot.intake.turnIntakeOff();

                        currentMovement = Movement.IDLE;

                    } else {
                        if (currentState != State.DROPPING_OFF && ballsCollected > 9) {
                            //if the robot has collected enough targets, head home if not idle or dropping off
                            currentState = State.HEADING_HOME;

                        } else {
                            switch (currentState) {
                                case LOCATING:
                                    //locates tennis balls
                                    robot.intake.intakeVariablePower(RESTING_INTAKE_POWER);

                                    //if we are not stopping rotation
                                    if (timer1.seconds() > 1.5) {
                                        //if there is a detection
                                        if (robot.cameraControls.currentRecognitions1.size() > 0 || robot.cameraControls.currentRecognitions2.size() > 0) {
                                            //stop the robot
                                            robot.drivetrain.breakFollowingImmediately();

                                            if (firstDetection) {
                                                firstDetection = false;
                                                timer1.reset();
                                                sleep(50);

                                            } else {
                                                firstDetection = true;
                                                //save the recognition data for the highest confidence recognition
                                                robot.cameraControls.saveRecognitionData();

                                                currentState = State.COLLECTING;

                                                //if the distance seems off
                                                if (savedDistance < 0 || savedDistance > 800) {
                                                    telemetry.addData("Recognition distance too far", time);
                                                    robot.drivetrain.simpleRotate(20);
                                                    firstAuto = true;
                                                    currentMode = Mode.FIRST_AUTO_CONTROL;
                                                    currentState = State.LOCATING;
                                                }

                                            }

                                            break;

                                        } else if (!robot.drivetrain.isSimpleRotating()) {
                                            firstDetection = true;
                                            //if there is no detection
                                            if (locateRotations < 300) {
                                                //rotate to find target
                                                robot.drivetrain.simpleRotate(30); //rotate 50 degrees counterclockwise
                                                locateRotations++;

                                                timer1.reset();
                                                sleep(50);

                                                currentMovement = Movement.LOCATING_BY_ROTATING;

                                            } else if (!robot.drivetrain.isSimpleStraight()){
                                                //drive to a new location asynchronously
                                                findNewLocation();
                                                driveToNewLocation();

                                                //once done: locateRotations = 0;

                                                currentMovement = Movement.LOCATING_BY_DRIVING;

                                            }

                                        }

                                        firstDetection = true;

                                    }

                                    break;

                                case COLLECTING:
                                    //~4 is the end of the collector
                                    if (!robot.drivetrain.isSimpleRotating()) {
                                        if (distance < 18 && !robot.drivetrain.isSimpleRotating() & collectRotations != 0) {
                                            telemetry.addData("Distance within 14in", time);
                                            robot.drivetrain.breakFollowingImmediately();
                                            //robot.drivetrain.update();

                                            //move backwards and turn
                                            Trajectory trajectory = robot.drivetrain.trajectoryBuilder(
                                                            robot.drivetrain.getPoseEstimate().plus(new Pose2d(0, 0, Math.toRadians(70))))
                                                    .back(36, Drivetrain.getVelocityConstraint(12.0, 0.2, TRACK_WIDTH),
                                                            Drivetrain.getAccelerationConstraint(6.0))
                                                    .build();

                                            robot.drivetrain.followTrajectory(trajectory);

                                            collectRotations = 0;
                                            collectStraights = 0;

                                            currentState = State.LOCATING;
                                            firstAuto = true;
                                            currentMode = Mode.FIRST_AUTO_CONTROL;

                                            break;

                                        } else if (!robot.drivetrain.isBusy()) {
                                            if (distance < 18 && !robot.drivetrain.isSimpleRotating() && collectRotations != 0) {
                                                telemetry.addData("Distance within 16in", time);
                                                robot.drivetrain.breakFollowingImmediately();

                                                //move backwards and turn
                                                Trajectory trajectory = robot.drivetrain.trajectoryBuilder(
                                                                robot.drivetrain.getPoseEstimate())
                                                        .forward(distance, Drivetrain.getVelocityConstraint(12, 1.0, TRACK_WIDTH),
                                                                Drivetrain.getAccelerationConstraint(4.0))
                                                        .build();

                                                robot.drivetrain.followTrajectoryAsync(trajectory);

                                                break;

                                            }

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

                                                } else {
                                                    //collected target
                                                    ballsCollected++;

                                                    collectRotations = 0;
                                                    collectStraights = 0;

                                                    currentState = State.LOCATING;
                                                    firstAuto = true;
                                                    currentMode = Mode.FIRST_AUTO_CONTROL;

                                                }
                                            }
                                        }

                                    }

                                    break;

                                case HEADING_HOME:
                                    robot.cameraControls.visionPortal1.setProcessorEnabled(robot.cameraControls.aprilTagProcessor1, true);
                                    robot.cameraControls.visionPortal2.setProcessorEnabled(robot.cameraControls.aprilTagProcessor2, true);

                                    //finds apriltag and then when found go to it
                                    robot.intake.intakeVariablePower(RESTING_INTAKE_POWER);

                                    currentMovement = Movement.FINDING_HOME;

                                    break;

                                case DROPPING_OFF:
                                    //dropping off the tennis balls when already at home
                                    robot.intake.turnIntakeOff();

                                    currentMovement = Movement.STATIONARY_FOR_DROPOFF;

                            }

                        }

                    }

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

            //sleep(300);

            //updates only the telemetry
            TelemetryControls.update();

            sleep(50);

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
