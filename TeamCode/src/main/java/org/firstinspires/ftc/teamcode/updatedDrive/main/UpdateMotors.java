package org.firstinspires.ftc.teamcode.updatedDrive.main;

import org.firstinspires.ftc.teamcode.updatedDrive.main.Intake;
import org.firstinspires.ftc.teamcode.updatedDrive.main.Robot;

public class UpdateMotors {
    private Robot robot;
    private Intake intake;

    public UpdateMotors(Robot rob, Intake in) {
        robot = rob;
        intake = in;
    }

    public void update() {
        if (intake.graduallyChangePower) {
            intake.updateIntake();;

        }

    }

}
