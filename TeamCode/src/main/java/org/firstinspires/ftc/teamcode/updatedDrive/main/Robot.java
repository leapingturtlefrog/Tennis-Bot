package org.firstinspires.ftc.teamcode.updatedDrive.main;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.updatedDrive.objects.Drivetrain;
import org.firstinspires.ftc.teamcode.updatedDrive.objects.GamepadControls;
import org.firstinspires.ftc.teamcode.updatedDrive.objects.Imu;
import org.firstinspires.ftc.teamcode.updatedDrive.objects.Intake;
import org.firstinspires.ftc.teamcode.updatedDrive.objects.REVDistanceSensor;


/**
 * This sets up the robot and
 * is how all other functions of the robot
 * are accessed
 */


public class Robot {
    //physical objects
    public Drivetrain drivetrain;

    public Intake intake;

    public Imu imu;

    public REVDistanceSensor distanceSensor;

    public GamepadControls gamepadControls;

    //VIrutal functions and processors

    public TelemetryControls telemetryControls;

    public TfodControls tfodControls;

    //public AprilTag aprilTag;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        drivetrain = new Drivetrain(hardwareMap, this);
        intake = new Intake(hardwareMap);
        imu = new Imu(hardwareMap);
        distanceSensor = new REVDistanceSensor(hardwareMap);
        gamepadControls = new GamepadControls(this);

        telemetryControls = new TelemetryControls(this, telemetry);

        tfodControls = new TfodControls(hardwareMap);
        //aprilTag = new AprilTag(hardwareMap);

    }

    public void update() {
        //update intake power if needed
        intake.updateIntake();

        //update distance sensor variable distance
        distanceSensor.updateDistance();

        //update the motors and their powers if need be

        //update telemetry and tfod
        telemetryControls.update(true);

    }

}
