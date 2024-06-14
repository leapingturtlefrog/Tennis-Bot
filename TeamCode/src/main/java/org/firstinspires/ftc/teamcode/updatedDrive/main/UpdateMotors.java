package org.firstinspires.ftc.teamcode.updatedDrive.main;

import org.firstinspires.ftc.teamcode.updatedDrive.main.Robot;

public class UpdateMotors {
    private Robot robot;

    public UpdateMotors(Robot rob) {
        robot = rob;

    }

    public void update() {
        //update intake power if needed
        robot.intake.updateIntake();

        //update distance sensor variable distance
        robot.distanceSensor.updateDistance();



    }

}
