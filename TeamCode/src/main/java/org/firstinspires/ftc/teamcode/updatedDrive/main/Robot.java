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
    //physical objects
    public Drivetrain drivetrain;

    public Intake intake;

    public Imu imu;

    public REVDistanceSensor distanceSensor;

    //VIrutal functions and processors
    public UpdateMotors updateMotors;

    public Telemetry telemetry;

    public Tfod tfod;

    public AprilTag aprilTag;

    public Robot(HardwareMap hardwareMap) {
        drivetrain = new Drivetrain(hardwareMap, this);
        intake = new Intake(hardwareMap);
        imu = new Imu(hardwareMap);
        distanceSensor = new REVDistanceSensor(hardwareMap);

        updateMotors = new UpdateMotors(this);

        telemetry = new Telemetry();

        tfod = new Tfod(hardwareMap);

        aprilTag = new AprilTag(hardwareMap);

    }

}
