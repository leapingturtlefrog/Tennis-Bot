package org.firstinspires.ftc.teamcode.updatedDrive4.main;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.updatedDrive4.objects.REVDistanceSensor;
import org.firstinspires.ftc.teamcode.updatedDrive4.objects.Drivetrain;
import org.firstinspires.ftc.teamcode.updatedDrive4.objects.GamepadControls;
import org.firstinspires.ftc.teamcode.updatedDrive4.objects.Imu;
import org.firstinspires.ftc.teamcode.updatedDrive4.objects.Intake;


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

    //Virtual functions and processorsf
    public TelemetryControls telemetryControls;

    public CameraControls cameraControls;

    //public AprilTag aprilTag;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepadUno) throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap, this);
        intake = new Intake(hardwareMap);
        imu = new Imu(hardwareMap);
        distanceSensor = new REVDistanceSensor(hardwareMap);
        gamepadControls = new GamepadControls(this, gamepadUno);

        telemetryControls = new TelemetryControls(this, telemetry);

        cameraControls = new CameraControls(hardwareMap);

        //aprilTag = new AprilTag(hardwareMap);

    }

    public void update() {
        //update roadrunner
        drivetrain.update();

        //update intake power if needed
        intake.updateIntake();

        //update distance sensor variable distance
        distanceSensor.updateDistance();

        //update the motors and their powers if need be

        //update telemetry and tfod
        telemetryControls.update();

    }

}
