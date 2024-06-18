package org.firstinspires.ftc.teamcode.updatedDrive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.updatedDrive.main.Robot;
import org.firstinspires.ftc.teamcode.updatedDrive.storage.PoseStorage;
import org.firstinspires.ftc.teamcode.updatedDrive.storage.Positions;

import static org.firstinspires.ftc.teamcode.updatedDrive.storage.Positions.startPoseEnumerated;
import static org.firstinspires.ftc.teamcode.updatedDrive.storage.Positions.currentMode;
import static org.firstinspires.ftc.teamcode.updatedDrive.storage.Positions.currentMovement;
import static org.firstinspires.ftc.teamcode.updatedDrive.storage.Positions.currentState;
import static org.firstinspires.ftc.teamcode.updatedDrive.storage.Positions.StartPoseEnumerated;
import static org.firstinspires.ftc.teamcode.updatedDrive.storage.Positions.Mode;
import static org.firstinspires.ftc.teamcode.updatedDrive.storage.Positions.Movement;
import static org.firstinspires.ftc.teamcode.updatedDrive.storage.Positions.State;
import static org.firstinspires.ftc.teamcode.updatedDrive.storage.PoseStorage.poseEstimate;




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


@TeleOp(group = "APushBot")
//@Disabled
public class Main extends LinearOpMode {
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
                    robot.telemetryControls.add("Main", "Driver control", time);
                    //gamepad controls that only occur if the mode is driver control
                    robot.gamepadControls.driverControlControls();

            }

            robot.telemetryControls.update();


        }

    }

}
