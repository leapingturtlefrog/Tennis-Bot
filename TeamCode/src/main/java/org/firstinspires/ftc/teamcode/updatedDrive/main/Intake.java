package org.firstinspires.ftc.teamcode.updatedDrive.main;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.updatedDrive.constants.DriveConstants.INTAKE_GRADUAL_BASE;
import static org.firstinspires.ftc.teamcode.updatedDrive.constants.DriveConstants.INTAKE_GRADUAL_POW;



/**
 * Class for the intake
 * Positive power is pulling in
 * And negative power is pushing out
 */


public class Intake {
    public DcMotorEx intake;

    //is the intake running without controller input constantly?
    private boolean intakeRunningContinuously;

    public boolean graduallyChangePower;

    private double intakeTargetPower, intakeStartPower, targetTime, a, c;

    ElapsedTime timer = new ElapsedTime();


    public Intake(HardwareMap hwMap) { init(hwMap); }

    //initialize intake
    private void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotorEx.class, "Intake");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        intakeRunningContinuously = false;
        graduallyChangePower = false;

    }

    public void turnIntakeOn() {
        graduallyChangePower(1.0);
        intakeRunningContinuously = true;

    }

    public void turnIntakeOff() {
        intake.setPower(0.0);
        intakeRunningContinuously = false;

    }

    //turn the intake to a variable power that is between -1 and 1
    public void intakeVariablePower(double power) {
        intakeRunningContinuously = false;
        graduallyChangePower(power);

    }

    //eq: a * BASE^(POW*time) + c = speed
    //eq solution: (log[(speed - c) / a]) / (log(b) * POW) = time
    private void graduallyChangePower(double targetPower) {
        intakeTargetPower = targetPower;
        intakeStartPower = intake.getPower();
        graduallyChangePower = true;

        if (intakeTargetPower == 0) {
            intake.setPower(0.0);
            graduallyChangePower = false;

        } else if (intakeTargetPower > intakeStartPower) {
            if (intakeStartPower == 0) {
                intake.setPower(0.01);
                intakeStartPower = 0.01;

            }



        } else if (intakeTargetPower < intakeStartPower) {
            if (intakeStartPower > 0) {


            }




        } else {
            graduallyChangePower = false;

        }

        if (intakeStartPower < 0.01) {
            intake.setPower(0.01);
            intakeStartPower = 0.01;

        }

        targetTime = (Math.log(0.99 / intakeStartPower)) / (INTAKE_GRADUAL_POW * Math.log(INTAKE_GRADUAL_BASE));

        timer.reset();

    }

    public void updateIntake() {
        double time = timer.seconds();

        if (time >= targetTime) {
            intake.setPower(1.0);
            graduallyChangePower = false;

        } else {
            intake.setPower(Math.round(intakeStartPower * Math.pow(INTAKE_GRADUAL_BASE, INTAKE_GRADUAL_POW * time) * 100.0) / 100.0);

        }

    }

}