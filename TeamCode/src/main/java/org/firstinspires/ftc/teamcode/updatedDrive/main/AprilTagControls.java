package org.firstinspires.ftc.teamcode.updatedDrive.main;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagControls {

    private AprilTagProcessor aprilTag;

    public AprilTagControls(HardwareMap hardwareMap) {
        initAprilTag();

    }

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        /*
        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

*/
    }
}
