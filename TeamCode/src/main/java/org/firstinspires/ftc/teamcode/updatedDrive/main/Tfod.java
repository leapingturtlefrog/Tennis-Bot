package org.firstinspires.ftc.teamcode.updatedDrive.main;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class Tfod {
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    private List<Recognition> currentRecognitions;
    private int currentDetectionIndex = -1;
    //for the recognitions we are heading to
    private List<Recognition> savedRecognitions;
    private int savedDetectionIndex;
    private double savedHeadingError;
    private double savedDistance;
    private double savedX, savedY;

    public Tfod(HardwareMap hardwareMap) {}



}
