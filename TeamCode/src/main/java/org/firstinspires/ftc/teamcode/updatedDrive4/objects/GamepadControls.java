package org.firstinspires.ftc.teamcode.updatedDrive4.objects;

import static org.firstinspires.ftc.teamcode.updatedDrive4.constants.Constants.INTAKE_START_POWER;
import static org.firstinspires.ftc.teamcode.updatedDrive4.opmode.Main.collectRotations;
import static org.firstinspires.ftc.teamcode.updatedDrive4.opmode.Main.collectStraights;
import static org.firstinspires.ftc.teamcode.updatedDrive4.opmode.Main.locateRotations;
import static org.firstinspires.ftc.teamcode.updatedDrive4.storage.Positions.Mode;
import static org.firstinspires.ftc.teamcode.updatedDrive4.storage.Positions.Movement;
import static org.firstinspires.ftc.teamcode.updatedDrive4.storage.Positions.State;
import static org.firstinspires.ftc.teamcode.updatedDrive4.storage.Positions.currentMode;
import static org.firstinspires.ftc.teamcode.updatedDrive4.storage.Positions.currentMovement;
import static org.firstinspires.ftc.teamcode.updatedDrive4.storage.Positions.currentState;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.updatedDrive4.main.Robot;


/**
 * The gamepad
 *
 *
 * //GAMEPAD1 CONTROLS
 *
 * Universal
 * x    switch mode to driver control, intake off, stop robot
 * y    switch mode to auto control, intake off
 * a    switch state to locate
 * b    switch state to idle
 *
 * Driver control
 * left bumper      intake off
 * right bumper     intake on
 * left trigger     intake reverse with variable speed
 * right trigger    intake collect with variable speed
 *
 * GAMEPAD2 CONTROLS
 *
 */


public class GamepadControls {

    private Robot robot;

    private Gamepad gamepad1, gamepad2;

    public GamepadControls(Robot rob, Gamepad gamepadUno, Gamepad gamepadDos) {
        robot = rob;
        gamepad1 = gamepadUno;
        gamepad2 = gamepadDos;
    }

    //allows the gamepad to change mode and state
    public void universalControls() {
        if (gamepad1.x) {
            //robot.intake.turnIntakeOff();
            //robot.drivetrain.setMotorPowers(0, 0, 0, 0);
            robot.drivetrain.breakFollowingSmooth();
            currentMode = Mode.DRIVER_CONTROL;
            currentMovement = Movement.DRIVER_IN_CONTROL;
            collectRotations = 0;
            collectStraights = 0;
            locateRotations = 0;

        } else if (gamepad1.y) {
            //robot.intake.turnIntakeOff();
            currentMode = Mode.FIRST_AUTO_CONTROL;
            currentMovement = Movement.IDLE;

        } else if (gamepad1.a) {
            currentState = State.LOCATING;

        } else if (gamepad1.b) {
            robot.drivetrain.breakFollowingSmooth();
            robot.intake.turnIntakeOff();
            currentState = State.IDLE;

        }


        if (gamepad2.dpad_right) {
            robot.cameraControls.visionPortal1.resumeLiveView();

        } else if (gamepad2.dpad_left) {
            robot.cameraControls.visionPortal1.stopLiveView();

        } else if (gamepad2.dpad_up) {
            robot.cameraControls.visionPortal2.resumeLiveView();

        } else if (gamepad2.dpad_down) {
            robot.cameraControls.visionPortal2.stopLiveView();

        }

        if (gamepad2.b) {
            robot.cameraControls.visionPortal1.resumeStreaming();

        } else if (gamepad2.x) {
            robot.cameraControls.visionPortal1.stopStreaming();

        } else if (gamepad2.y) {
            robot.cameraControls.visionPortal2.resumeStreaming();

        } else if (gamepad2.a) {
            robot.cameraControls.visionPortal2.stopStreaming();

        }

    }

    //allows the gamepad to do certain controls during the driver period
    public void driverControlControls() {
        //used for driver control-specific commands
        robot.drivetrain.setWeightedDrivePower(
                new Pose2d(
                        (gamepad1.dpad_up ? 1 : (gamepad1.dpad_down ? -1 : -gamepad1.left_stick_y)),
                        (gamepad1.dpad_left ? 1 : (gamepad1.dpad_right ? -1 : -gamepad1.left_stick_x)),
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
            INTAKE_START_POWER += 0.001;
        } else if (gamepad1.dpad_down) {
            INTAKE_START_POWER -= 0.001;
        }


    }
}
