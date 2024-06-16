package org.firstinspires.ftc.teamcode.updatedDrive.main;

import static org.firstinspires.ftc.teamcode.updatedDrive.storage.PoseStorage.poseEstimate;
import static org.firstinspires.ftc.teamcode.updatedDrive.storage.Positions.currentMode;
import static org.firstinspires.ftc.teamcode.updatedDrive.storage.Positions.currentMovement;
import static org.firstinspires.ftc.teamcode.updatedDrive.storage.Positions.currentState;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class TelemetryControls{

    private Robot robot;

    private Telemetry telemetry;

    public TelemetryControls(Robot rob, Telemetry telem) {
        robot = rob;
        telemetry = telem;

    }

    public void update(boolean tfodAndAprilTag) {
        //modes, states, and movement
        telemetry.addData("mode", currentMode);
        telemetry.addData("state", currentState);
        telemetry.addData("movement", currentMovement);

        //pose

        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("", "---------------"); //15 dashes*/

        if (tfodAndAprilTag) {
            //tfod
            updateTfodTelemAndDetectionIndex();

        }

        telemetry.update();

    }

    //updates the telemetry for the tfod and the
    public void updateTfodTelemAndDetectionIndex() {
        robot.tfodControls.currentRecognitions = robot.tfodControls.myTfodProcessor.getRecognitions();

        telemetry.addData("# objects detected", robot.tfodControls.currentRecognitions.size());

        double highestConfidence = 0;
        int index = -1;
        robot.tfodControls.currentDetectionIndex = -1;

        //display info on each recognition
        for (Recognition recognition : robot.tfodControls.currentRecognitions) {
            if (recognition.getConfidence() > highestConfidence) {
                highestConfidence = recognition.getConfidence();
                robot.tfodControls.currentDetectionIndex = index;

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
}
