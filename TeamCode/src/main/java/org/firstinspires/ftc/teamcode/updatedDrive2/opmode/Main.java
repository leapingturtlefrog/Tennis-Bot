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

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
                                    saveRecognitionData();

                                    currentState = State.COLLECTING;

                                } else if (!robot.drivetrain.isSimpleRotating()) {
                                    //if there is no detection
                                    if (locateRotations < 36) {
                                        //rotate to find target
                                        robot.drivetrain.simpleRotate(10.0); //rotate 10 degrees counterclockwise
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
                                if (!robot.drivetrain.isSimpleRotating() && collectRotations == 0) {
                                    //needs to rotate to target
                                    robot.drivetrain.simpleRotate(-savedHeadingError);
                                    collectRotations++;

                                    currentMovement = Movement.ROTATING_TO_TARGET;

                                } else if (!robot.drivetrain.isSimpleStraightMinusEnd() && collectStraights == 0) {
                                    //needs to collect target
                                    robot.intake.intakeVariablePower(1.0);

                                    robot.drivetrain.simpleStraight(savedDistance + 3, 0);
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
                        currentMode = Mode.AUTO_CONTROL;
                        currentState = State.LOCATING;

                    }

            }

            //updates only the telemetry
            robot.telemetryControls.update();


        }

    }


    /** FUNCTIONS ***/


    //save the recognition data for the highest confidence recognition
    private void saveRecognitionData() {
        savedRecognitions = robot.tfodControls.currentRecognitions;

        savedDetectionIndex = robot.tfodControls.currentDetectionIndex;
        savedHeadingError = savedRecognitions.get(savedDetectionIndex).estimateAngleToObject(AngleUnit.DEGREES);

        savedX = (savedRecognitions.get(savedDetectionIndex).getLeft() + savedRecognitions.get(savedDetectionIndex).getRight()) / 2.0;
        savedY = (savedRecognitions.get(savedDetectionIndex).getTop() + savedRecognitions.get(savedDetectionIndex).getBottom()) / 2.0;

        /* WRONG, does y value based on distance
        //approximates the distance based on the y value
        //552 + -3.69x + 0.0245x^2 + -7.77E-05x^3 + 9.57E-08x^4
        savedDistance = 552 - 3.69*savedY + 0.0245*savedY*savedY - 0.0000777*Math.pow(savedY, 3) + 0.0000000957*Math.pow(savedY, 4);*/

        //Correct, produces distance based on y value //update: produces near-zero values
        //1.31E+07 + -214233x + 1461x^2 + -5.31x^3 + 0.0109x^4 + -1.18E-05x^5 + 5.36E-09x^6
        //savedDistance = 13100000 - 214233*savedY + 1461*savedY*savedY - 5.31*Math.pow(savedY, 3) + 0.0109*Math.pow(savedY, 4) - 0.000018*Math.pow(savedY, 5) + 0.00000000536*Math.pow(savedY, 6);

        //212101 + -2265x + 9.08x^2 + -0.0162x^3 + 1.08E-05x^4
        //savedDistance = 212101 - 2265*savedY + 9.08*savedY*savedY - 0.0162*Math.pow(savedY, 3) + 0.0000108*Math.pow(savedY, 4);

        /*
        BigDecimal y = new BigDecimal(savedY);

        BigDecimal a1 = new BigDecimal(212101);

        BigDecimal a2a = new BigDecimal(-2265);
        BigDecimal a2 = a2a.multiply(y);

        BigDecimal a3a = new BigDecimal(9.08);
        BigDecimal a3b = y.pow(2);
        BigDecimal a3 = a3a.multiply(a3b);

        BigDecimal a4a = new BigDecimal(-0.0162);
        BigDecimal a4b = y.pow(3);
        BigDecimal a4 = a4a.multiply(a4b);

        BigDecimal a5a = new BigDecimal(0.0000108);
        BigDecimal a5b = y.pow(4);
        BigDecimal a5 = a5a.multiply(a5b);

        BigDecimal ans = a1.add(a2.add(a3.add(a4.add(a5))));

        savedDistance = ans.doubleValue();*/

        //ab^y + c WRONG
        //a=239.735, b=0.983259, c=315.151
        //savedDistance = 239.735*Math.pow(0.983259, savedY) + 315.151;


        //ab^y + c
        //a=7.5252e7, b=0.960118, c=60.4614
        savedDistance = (7.5252E7) * (Math.pow(0.960118, savedY)) + 60.4614;

        if (savedDistance < 30 || savedDistance > 300) {
            robot.drivetrain.simpleRotate(360);
            currentMode = Mode.FIRST_AUTO_CONTROL;
            currentState = State.LOCATING;
        }

    }

    //TODO: finish these functions
    //find new location using virtual map
    private void findNewLocation() {

    }

    //drives to new location avoiding objects
    private void driveToNewLocation() {

    }

}
