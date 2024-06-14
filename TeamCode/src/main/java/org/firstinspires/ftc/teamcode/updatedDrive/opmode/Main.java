package org.firstinspires.ftc.teamcode.updatedDrive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    Pose2d poseEstimate;

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        //robot.tfod.initTfod();

        startPoseEnumerated = StartPoseEnumerated.COURT_FOUR;
        startPose = Positions.getStartPose();

        robot.drivetrain.setPoseEstimate(startPose);
        PoseStorage.currentPose = robot.drivetrain.getPoseEstimate();



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            //updates the drivetrain, distance sensor, tfod, AprilTags, etc
            //Includes telemetry update
            robot.update();

            poseEstimate = robot.drivetrain.getPoseEstimate();

            robot.gamepadControls.universalControls();

            switch (currentMode) {
                case AUTO_CONTROL:

                    break;

                case DRIVER_CONTROL:
                    robot.gamepadControls.driverControlControls();

            }



            //only updates the telemetry
            robot.telemetry.update();

        }

    }


}
