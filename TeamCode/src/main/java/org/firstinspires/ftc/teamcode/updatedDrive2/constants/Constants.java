package org.firstinspires.ftc.teamcode.updatedDrive2.constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.Arrays;


/**
 * The constants for the drivetrain
 * In INCHES
 */


@Config
public class Constants {

    /** DRIVETRAIN */

    //motor specifications
    public static final double TICKS_PER_REV = 751.8; //GoBuilda 5202 26.9:1 ratio
    public static final double MAX_RPM = 223; //GoBuilda 5202 26.9:1 ratio

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER = false; //Leave this as FALSE or else drivetrain didn't work
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    //physical constants
    public static double WHEEL_RADIUS = 2;
    public static double GEAR_RATIO = 0.933; //output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 17.5;

    /** DRIVING PARAMETERS */

    //The feedforward parameters used to model the drive motor behavior
    public static double kV = 0.0237;
    public static double kA = 0.0035;
    public static double kStatic = 0.002;

    //PIDCoefficients
    public static double KP_TRANSLATIONAL_PID = 0;
    public static double KP_HEADING_PID = 6;

    //values used to constrain the robot's movement
    public static double MAX_VEL = 35;
    public static double MAX_ACCEL = 35;
    public static double MAX_ANG_VEL = 2.11;
    public static double MAX_ANG_ACCEL = 1.5;

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0); //0,0,0 //may need to be 0,0,0 if path is wonky
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(6, 0, 0); //0,0,0

    public static double LATERAL_MULTIPLIER = 1.560; //-1.560; //1.460; //0.683; //1;
    //Test if negative lateral multiplier works //update: it makes the strafing backwards

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    public static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    /** IMU */

    //imu constants
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

    /** INTAKE */

    //Motor gradually changing power constants (can graph in Geogebra)
    public static double INTAKE_GRADUAL_BASE = 2.7; //higher and the longer it takes
    public static double INTAKE_GRADUAL_POW = 4.0; //higher and the shorter it takes

    public static double INTAKE_START_POWER = 0.0;

    public static double RESTING_INTAKE_POWER = 0.5; //the power for when the robot
    //is not collecting a target


    /** Tfod Image detection */

    public static final String TFOD_MODEL_FILE =
            "/sdcard/FIRST/tflitemodels/TennisBallModel2.tflite";
    //labels of model object
    public static final String[] LABELS = {
            "a"
    };


    /*** Functions ***/

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

}
