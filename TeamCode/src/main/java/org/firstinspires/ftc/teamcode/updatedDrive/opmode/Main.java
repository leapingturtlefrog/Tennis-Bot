package org.firstinspires.ftc.teamcode.updatedDrive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


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
    //2 modes for if the driver is controlling the robot or auto is
    enum Mode {
        DRIVER_CONTROL,
        AUTO_CONTROL

    }
    Mode currentMode = Mode.DRIVER_CONTROL;

    //states for which period of auto control is occuring

    @Override
    public void runOpMode() throws InterruptedException {


    }


}
