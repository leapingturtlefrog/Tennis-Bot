package org.firstinspires.ftc.teamcode.updatedDrive3.main;

import static org.firstinspires.ftc.teamcode.updatedDrive3.constants.Constants.INTAKE_START_POWER;
import static org.firstinspires.ftc.teamcode.updatedDrive3.main.CameraControls.maxExposure;
import static org.firstinspires.ftc.teamcode.updatedDrive3.main.CameraControls.maxGain;
import static org.firstinspires.ftc.teamcode.updatedDrive3.main.CameraControls.minExposure;
import static org.firstinspires.ftc.teamcode.updatedDrive3.main.CameraControls.minGain;
import static org.firstinspires.ftc.teamcode.updatedDrive3.main.CameraControls.myExposure;
import static org.firstinspires.ftc.teamcode.updatedDrive3.main.CameraControls.myGain;
import static org.firstinspires.ftc.teamcode.updatedDrive3.main.CameraControls.savedDetectionIndex;
import static org.firstinspires.ftc.teamcode.updatedDrive3.main.CameraControls.savedDistance;
import static org.firstinspires.ftc.teamcode.updatedDrive3.main.CameraControls.savedHeadingError;
import static org.firstinspires.ftc.teamcode.updatedDrive3.main.CameraControls.savedRecognitions;
import static org.firstinspires.ftc.teamcode.updatedDrive3.main.CameraControls.savedX;
import static org.firstinspires.ftc.teamcode.updatedDrive3.main.CameraControls.savedY;
import static org.firstinspires.ftc.teamcode.updatedDrive3.storage.PoseStorage.poseEstimate;
import static org.firstinspires.ftc.teamcode.updatedDrive3.storage.Positions.currentMode;
import static org.firstinspires.ftc.teamcode.updatedDrive3.storage.Positions.currentMovement;
import static org.firstinspires.ftc.teamcode.updatedDrive3.storage.Positions.currentState;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

public class TelemetryControls{

    private Robot robot;

    private Telemetry telemetry;

    private Map<String, Object[] > addedTelemetry = new HashMap<>();

    public TelemetryControls(Robot rob, Telemetry telem) {
        robot = rob;
        telemetry = telem;

    }

    public void update() {
        //distance sensor
        telemetry.addData("sensorDistance", robot.distanceSensor.getDistance());

        //modes, states, and movement
        telemetry.addData("mode", currentMode);
        telemetry.addData("state", currentState);
        telemetry.addData("movement", currentMovement);

        //pose

        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());

        telemetry.addData("Added telemetry", "");

        //addedTelemetry
        telemetry.addData("INTAKE_START_POWER", INTAKE_START_POWER);
        for (String key : addedTelemetry.keySet()) {
            telemetry.addData(key, " value - "
                    + Objects.requireNonNull(addedTelemetry.get(key))[0] +
                    ", time - " + Objects.requireNonNull(addedTelemetry.get(key))[1]);

        }

        //current saved recognition and its data
        if (savedRecognitions != null && savedRecognitions.size() > 0) {
            telemetry.addData("Saved object", "");
            telemetry.addData("- Detection index", savedDetectionIndex);
            telemetry.addData("- Heading error", savedHeadingError);
            telemetry.addData("- x", savedX);
            telemetry.addData("- y", savedY);
            telemetry.addData("- Distance", savedDistance);
        }

        //the rest of the objects detected
        telemetry.addData("", "---------------"); //15 dashes

        updateTfodTelemAndDetectionIndex();

        //additional info
        telemetry.addData("", "---------------"); //15 dashes
        telemetry.addData("Exposure","%d  (%d - %d)", myExposure, minExposure, maxExposure);
        telemetry.addData("Gain","%d  (%d - %d)", myGain, minGain, maxGain);

        telemetry.update();

    }

    //updates the telemetry for the tfod and the
    public void updateTfodTelemAndDetectionIndex() {
        robot.cameraControls.currentRecognitions = robot.cameraControls.myTfodProcessor.getRecognitions();

        telemetry.addData("# objects detected", robot.cameraControls.currentRecognitions.size());

        double highestConfidence = 0;
        int index = 0;
        robot.cameraControls.currentDetectionIndex = -1;

        //display info on each recognition
        for (Recognition recognition : robot.cameraControls.currentRecognitions) {
            if (recognition.getConfidence() > highestConfidence) {
                highestConfidence = recognition.getConfidence();
                robot.cameraControls.currentDetectionIndex = index;

            }

            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% conf.)",
                    recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f, %.0f", x, y);
            telemetry.addData("- Angle", recognition.estimateAngleToObject(AngleUnit.DEGREES));
            telemetry.addData("- Size", "%.0f x %.0f",
                    recognition.getWidth(), recognition.getHeight());

            index++;
        }
    }

    //add custom messages
    public void add(String name, String message, Double time) {
        addedTelemetry.put(name, new Object[]{message, time});

    }
}
