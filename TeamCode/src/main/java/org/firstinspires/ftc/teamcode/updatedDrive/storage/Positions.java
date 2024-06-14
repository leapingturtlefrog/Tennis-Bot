package org.firstinspires.ftc.teamcode.updatedDrive.storage;


/**
 * Holds the different modes, states, and movements
 * of the robot
 */


public class Positions {
    //2 modes for if the driver is controlling the robot
    //or auto is
    enum Mode {
        DRIVER_CONTROL,
        AUTO_CONTROL

    }
    public static Mode currentMode;

    //states for which period of auto control is occurring
    enum State {
        IDLE,
        LOCATING,
        COLLECTING,
        HEADING_HOME, //heading home
        DROPPING_OFF

    }
    public static State currentState;

    //to keep track of the specific movement that
    //is currently being performed
    enum Movement {
        IDLE,
        LOCATING_BY_ROTATING,
        LOCATING_BY_DRIVING,
        ROTATING_TO_TARGET,
        DRIVING_STRAIGHT_TO_TARGET,
        COLLECTING_TARGET,
        FINDING_HOME,
        DRIVING_HOME,
        STATIONARY_FOR_DROPOFF,
        DRIVER_IN_CONTROL

    }
    public static Movement currentMovement;

    //to determine where the robot is to start
    enum StartPoseEnumerated {
        COURT_ONE,
        COURT_TWO,
        COURT_THREE,
        COURT_FOUR

    }
    public static StartPoseEnumerated startPoseEnumerated;

}
