package org.firstinspires.ftc.teamcode.updatedDrive.objects;

import org.firstinspires.ftc.teamcode.updatedDrive.main.Robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import static org.firstinspires.ftc.teamcode.updatedDrive.storage.Positions.startPoseEnumerated;
import static org.firstinspires.ftc.teamcode.updatedDrive.storage.Positions.currentMode;
import static org.firstinspires.ftc.teamcode.updatedDrive.storage.Positions.currentMovement;
import static org.firstinspires.ftc.teamcode.updatedDrive.storage.Positions.currentState;
import static org.firstinspires.ftc.teamcode.updatedDrive.storage.Positions.StartPoseEnumerated;
import static org.firstinspires.ftc.teamcode.updatedDrive.storage.Positions.Mode;
import static org.firstinspires.ftc.teamcode.updatedDrive.storage.Positions.Movement;
import static org.firstinspires.ftc.teamcode.updatedDrive.storage.Positions.State;


/**
 * The gamepad
 *
 *
 * //GAMEPAD CONTROLS
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
 */


public class GamepadControls extends LinearOpMode {

    private Robot robot;

    private Gamepad gamepad;

    public GamepadControls(Robot rob) { robot = rob; gamepad = new Gamepad(); }

    @Override
    public void runOpMode() throws InterruptedException {

    }

    //allows the gamepad to change mode and state
    public void universalControls() {
        if (gamepad1.x) {
            robot.intake.turnIntakeOff();
            robot.drivetrain.setMotorPowers(0, 0, 0, 0);
            currentMode = Mode.DRIVER_CONTROL;
            currentMovement = Movement.DRIVER_IN_CONTROL;

        } else if (gamepad1.y) {
            robot.intake.turnIntakeOff();
            currentMode = Mode.AUTO_CONTROL;
            currentMovement = Movement.IDLE;

        } else if (gamepad1.a) {
            currentState = State.LOCATING;

        } else if (gamepad1.b) {
            robot.intake.turnIntakeOff();
            currentState = State.IDLE;

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


    }
}
