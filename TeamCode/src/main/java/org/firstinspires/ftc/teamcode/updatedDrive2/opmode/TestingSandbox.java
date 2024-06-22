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
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.updatedDrive2.main.Robot;
import org.firstinspires.ftc.teamcode.updatedDrive2.storage.PoseStorage;
import org.firstinspires.ftc.teamcode.updatedDrive2.storage.Positions;


/**
 * Used to test functions. Auto control removed
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


@TeleOp(name="testingSandbox", group = "APushBot")
//@Disabled
public class TestingSandbox extends LinearOpMode {
    private int locateRotations = 0;
    private int collectRotations = 0;
    private int collectStraights = 0;

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

                    break;

                case DRIVER_CONTROL:
                    //gamepad controls that only occur if the mode is driver control
                    robot.drivetrain.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    //if triggers are 0 turn off intake if it is not continuous
                    if (gamepad1.left_trigger < 0.1 && gamepad1.right_trigger < 0.1 && !robot.intake.isIntakeRunningContinuously()) {
                        robot.intake.turnIntakeOff();
                    }

                    if (gamepad1.left_bumper) {
                        robot.intake.turnIntakeOff();

                    } else if (gamepad1.right_bumper) {
                        robot.intake.turnIntakeOn();

                    } else if (gamepad1.left_trigger > 0.1) {
                        robot.intake.intakeVariablePower(-gamepad1.left_trigger);

                    } else if (gamepad1.right_trigger > 0.1) {
                        robot.intake.intakeVariablePower(gamepad1.right_trigger);

                    }

                    if (gamepad1.dpad_up) {
                        Trajectory forwardLastBit = robot.drivetrain.trajectoryBuilder(new Pose2d())
                                .forward(12)
                                .build();

                        robot.drivetrain.followTrajectory(forwardLastBit);

                    } else if (gamepad1.dpad_down) {
                        Trajectory forwardLastBit = robot.drivetrain.trajectoryBuilder(new Pose2d())
                                .forward(100)
                                .build();

                        robot.drivetrain.followTrajectory(forwardLastBit);

                    } else if (gamepad1.dpad_left) {
                        robot.drivetrain.turn(Math.toRadians(90));

                    } else if (gamepad1.dpad_right) {
                        robot.drivetrain.turn(Math.toRadians(-180));

                    }

                    currentMovement = Movement.DRIVER_IN_CONTROL;

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

        //approximates the distance based on the y value
        //552 + -3.69x + 0.0245x^2 + -7.77E-05x^3 + 9.57E-08x^4
        savedDistance = 552 - 3.69*savedY + 0.0245*savedY*savedY - 0.0000777*Math.pow(savedY, 3) + 0.0000000957*Math.pow(savedY, 4);

    }

    //TODO: finish these functions
    //find new location using virtual map
    private void findNewLocation() {

    }

    //drives to new location avoiding objects
    private void driveToNewLocation() {

    }

}
