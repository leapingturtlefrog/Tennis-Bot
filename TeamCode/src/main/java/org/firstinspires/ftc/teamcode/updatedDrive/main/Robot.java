package org.firstinspires.ftc.teamcode.updatedDrive.main;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.updatedDrive.objects.Drivetrain;
import org.firstinspires.ftc.teamcode.updatedDrive.objects.Imu;
import org.firstinspires.ftc.teamcode.updatedDrive.objects.Intake;
import org.firstinspires.ftc.teamcode.updatedDrive.objects.REVDistanceSensor;


/**
 * This sets up the robot and
 * is how all other functions of the robot
 * are accessed
 */


public class Robot {
    public Drivetrain drivetrain;

    public Intake intake;

    public Imu imu;

    public UpdateMotors updateMotors;

    public REVDistanceSensor distanceSensor;

    public Telemetry telemetry;

    public Robot(HardwareMap hardwareMap) {
        drivetrain = new Drivetrain(hardwareMap, this);
        intake = new Intake(hardwareMap);
        imu = new Imu(hardwareMap);
        distanceSensor = new REVDistanceSensor(hardwareMap);

        updateMotors = new UpdateMotors(this);

        telemetry = new Telemetry();

    }

}
