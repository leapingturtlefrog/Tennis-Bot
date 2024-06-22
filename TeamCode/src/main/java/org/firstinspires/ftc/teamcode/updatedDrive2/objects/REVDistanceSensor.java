package org.firstinspires.ftc.teamcode.updatedDrive2.objects;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * Class for the REV2mDistanceSensor
 */


public class REVDistanceSensor {
    public DistanceSensor distanceSensor;

    public static double distance;
    public static double savedDistance;

    public REVDistanceSensor(HardwareMap hardwareMap) {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

    }

    //updates the distance variable
    public void updateDistance() {
        distance = distanceSensor.getDistance(DistanceUnit.INCH);

    }

    public double getDistance() {
        updateDistance();
        return distance;

    }

}
