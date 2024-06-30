package org.firstinspires.ftc.teamcode.copyThatMayChange.main;

import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.INTAKE_START_POWER;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.exposure1;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.exposure2;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.gain1;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.gain2;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.maxExposure1;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.maxExposure2;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.maxGain1;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.maxGain2;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.minExposure1;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.minExposure2;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.minGain1;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.constants.Constants.minGain2;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.main.CameraControls.savedDetectionIndex;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.main.CameraControls.savedDistance;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.main.CameraControls.savedHeadingError;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.main.CameraControls.savedRecognitions;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.main.CameraControls.savedX;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.main.CameraControls.savedY;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.storage.PoseStorage.poseEstimate;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.storage.Positions.currentMode;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.storage.Positions.currentMovement;
import static org.firstinspires.ftc.teamcode.copyThatMayChange.storage.Positions.currentState;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

public class TelemetryControls{

    private static Robot robot;

    private static Telemetry telemetry;

    private static Map<String, Object[] > addedTelemetry = new HashMap<>();

    public TelemetryControls(Robot rob, Telemetry telem) {
        robot = rob;
        telemetry = telem;

    }

    public static void update() {
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
            telemetry.addData("- Detection index", savedDetectionIndex[0] + ", " + savedDetectionIndex[1]);
            telemetry.addData("- Heading error", savedHeadingError);
            telemetry.addData("- x", savedX);
            telemetry.addData("- y", savedY);
            telemetry.addData("- Distance", savedDistance);
        }

        telemetry.addData("vision1 fps", robot.cameraControls.visionPortal1.getFps());
        telemetry.addData("vision2 fps", robot.cameraControls.visionPortal2.getFps());

        telemetry.addData("Exposure1","%d  (%d - %d)", exposure1, minExposure1, maxExposure1);
        telemetry.addData("Gain1","%d  (%d - %d)", gain1, minGain1, maxGain1);
        telemetry.addData("Exposure2","%d  (%d - %d)", exposure2, minExposure2, maxExposure2);
        telemetry.addData("Gain2","%d  (%d - %d)", gain2, minGain2, maxGain2);

        //the rest of the objects detected
        telemetry.addData("", "---------------"); //15 dashes

        updateTfodTelemAndDetectionIndex();

        //additional info
        telemetry.addData("", "---------------"); //15 dashes

        telemetry.update();

    }

    //updates the telemetry for the tfod
    public static void updateTfodTelemAndDetectionIndex() {
        robot.cameraControls.currentRecognitions1 = robot.cameraControls.tfodProcessor1.getRecognitions();
        robot.cameraControls.currentRecognitions2 = robot.cameraControls.tfodProcessor2.getRecognitions();

        telemetry.addData("# objects detected1", robot.cameraControls.currentRecognitions1.size());
        telemetry.addData("# objects detected2", robot.cameraControls.currentRecognitions2.size());

        double highestConfidence = 0;
        int index = 0;
        robot.cameraControls.currentDetectionIndex = new int[]{-1, -1};

        //display info on each recognition
        for (Recognition recognition : robot.cameraControls.currentRecognitions1) {
            if (recognition.getConfidence() > highestConfidence) {
                highestConfidence = recognition.getConfidence();
                robot.cameraControls.currentDetectionIndex = new int[]{0, index};

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

        index = 0;

        for (Recognition recognition : robot.cameraControls.currentRecognitions2) {
            if (recognition.getConfidence() > highestConfidence) {
                highestConfidence = recognition.getConfidence();
                robot.cameraControls.currentDetectionIndex = new int[]{1, index};

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
    public static void add(String name, String message, Double time) {
        addedTelemetry.put(name, new Object[]{message, time});

    }

    public static void add(String name, String message) {
        telemetry.addData(name, message);
        telemetry.update();

    }

    public static void add(String name, double number) {
        telemetry.addData(name, number);
        telemetry.update();

    }

    public static void add(String message) {
        telemetry.addLine(message);
        telemetry.update();

    }
}
