package org.firstinspires.ftc.teamcode.updatedDrive.main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Class for the intake
 * Positive power is pulling in
 * And negative power is pushing out
 */


public class Intake {
    public DcMotorEx intake;

    //is the intake running without controller input constantly?
    private boolean isIntakeRunningContinuously;

    private double intakeTargetPower;


    public Intake(HardwareMap hwMap) { init(hwMap); }

    //initialize intake
    private void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotorEx.class, "Intake");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        isIntakeRunningContinuously = false;

    }

    public void turnIntakeOn() {
        intake.setPower(1.0);
        isIntakeRunningContinuously = true;

    }

    public void turnIntakeOff() {
        intake.setPower(0.0);
        isIntakeRunningContinuously = false;

    }

    //turn the intake to a variable power that is between -1 and 1
    public void intakeVariablePower(double power) {
        isIntakeRunningContinuously = false;
        graduallyChangePower(power);

    }

    private void graduallyChangePower(double targetPower) {
        intakeTargetPower = targetPower;

        double currentPower = intake.getPower();

    }

}
