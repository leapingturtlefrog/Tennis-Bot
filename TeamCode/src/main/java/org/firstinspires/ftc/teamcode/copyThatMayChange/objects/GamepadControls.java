package org.firstinspires.ftc.teamcode.copyThatMayChange.objects;

import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.exposure1;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.exposure2;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.gain1;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.gain2;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.maxExposure1;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.maxExposure2;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.maxGain1;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.maxGain2;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.minExposure1;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.minExposure2;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.minGain1;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.minGain2;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.opmode.Main.collectRotations;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.opmode.Main.collectStraights;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.opmode.Main.locateRotations;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.storage.Positions.Mode;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.storage.Positions.Movement;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.storage.Positions.State;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.storage.Positions.currentMode;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.storage.Positions.currentMovement;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.storage.Positions.currentState;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.copyThatMayChange.main.Robot;


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
 * Universal
 *
 *
 */


public class GamepadControls {

    private Robot robot;

    private Gamepad gamepad1, gamepad2;

    private boolean lastLeftTrigger2 = false, lastRightTrigger2 = false, lastLeftBumper2 = false, lastRightBumper2 = false;

    public GamepadControls(Robot rob, Gamepad gamepadUno, Gamepad gamepadDos) {
        robot = rob;
        gamepad1 = gamepadUno;
        gamepad2 = gamepadDos;
    }

    //allows the gamepad to change mode and state
    public void universalControls() throws InterruptedException {
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
            gain1 = Range.clip(gain1-10, minGain1, maxGain1);
            robot.cameraControls.setManualExposure(exposure1, gain1, robot.cameraControls.visionPortal1);

        } else if (gamepad2.dpad_left) {
            robot.cameraControls.visionPortal1.stopLiveView();

        } else if (gamepad2.dpad_up) {
            robot.cameraControls.visionPortal2.resumeLiveView();
            gain2 = Range.clip(gain2-10, minGain2, maxGain2);
            robot.cameraControls.setManualExposure(exposure2, gain2, robot.cameraControls.visionPortal2);

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

        boolean leftTrigger2 = gamepad2.left_trigger > 0.25, rightTrigger2 = gamepad2.right_trigger > 0.25, leftBumper2 = gamepad2.left_bumper, rightBumper2 = gamepad2.right_bumper;


        if (leftTrigger2 && !lastLeftTrigger2) {
            exposure1 = Range.clip(exposure1-1, minExposure1, maxExposure1);
            robot.cameraControls.setManualExposure(exposure1, gain1, robot.cameraControls.visionPortal1);

        } else if (leftBumper2 && !lastLeftBumper2) {
            exposure1 = Range.clip(exposure1+1, minExposure1, maxExposure1);
            robot.cameraControls.setManualExposure(exposure1, gain1, robot.cameraControls.visionPortal1);

        } else if (rightTrigger2 && !lastRightTrigger2) {
            exposure2 = Range.clip(exposure2-1, minExposure2, maxExposure2);
            robot.cameraControls.setManualExposure(exposure2, gain2, robot.cameraControls.visionPortal2);

        } else if (rightBumper2 && !lastRightBumper2) {
            exposure2 = Range.clip(exposure2+1, minExposure2, maxExposure2);
            robot.cameraControls.setManualExposure(exposure2, gain2, robot.cameraControls.visionPortal2);

        }

        lastLeftTrigger2 = leftTrigger2;
        lastRightTrigger2 = rightTrigger2;
        lastLeftBumper2 = leftBumper2;
        lastRightBumper2 = rightBumper2;

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

    }
}
