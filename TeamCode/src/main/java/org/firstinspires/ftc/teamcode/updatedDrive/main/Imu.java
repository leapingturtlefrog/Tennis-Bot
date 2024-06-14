package org.firstinspires.ftc.teamcode.updatedDrive.main;

import static org.firstinspires.ftc.teamcode.updatedDrive.constants.DriveConstants.LOGO_FACING_DIR;
import static org.firstinspires.ftc.teamcode.updatedDrive.constants.DriveConstants.USB_FACING_DIR;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Imu {

    public IMU imu;

    public Imu(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                LOGO_FACING_DIR, USB_FACING_DIR));
        imu.initialize(parameters);

        imu.resetYaw();

    }

    public double getRawExternalHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public Double getExternalHeadingVelocity() {
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

}