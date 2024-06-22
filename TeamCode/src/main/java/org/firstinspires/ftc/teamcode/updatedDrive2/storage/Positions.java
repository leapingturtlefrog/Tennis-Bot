package org.firstinspires.ftc.teamcode.updatedDrive2.storage;


import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Holds the different modes, states, and movements
 * of the robot
 */


public class Positions {
    //2 modes for if the driver is controlling the robot
    //or auto is
    public static enum Mode {
        DRIVER_CONTROL,
        FIRST_AUTO_CONTROL, //right after collecting a ball or first starting auto
        AUTO_CONTROL

    }
    public static Mode currentMode;

    //states for which period of auto control is occurring
    public static enum State {
        IDLE,
        LOCATING,
        COLLECTING,
        HEADING_HOME,
        DROPPING_OFF

    }
    public static State currentState;

    //to keep track of the specific movement that
    //is currently being performed
    public static enum Movement {
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
    public static enum StartPoseEnumerated {
        COURT_ONE,
        COURT_TWO,
        COURT_THREE,
        COURT_FOUR

    }
    public static StartPoseEnumerated startPoseEnumerated;

    //TODO: Change poses to match court positions
    public static Pose2d getStartPose() {
        switch (startPoseEnumerated) {
            case COURT_ONE:
                return new Pose2d(0,0,0);

            case COURT_TWO:
                return new Pose2d(0,0,0);

            case COURT_THREE:
                return new Pose2d(0,0,0);

            case COURT_FOUR:
                return new Pose2d(0,0,0);

            default:
                return new Pose2d(0, 0, 0);
        }

    }

}
