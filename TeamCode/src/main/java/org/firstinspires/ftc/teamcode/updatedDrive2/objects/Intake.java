package org.firstinspires.ftc.teamcode.updatedDrive2.objects;


import static org.firstinspires.ftc.teamcode.updatedDrive2.constants.Constants.INTAKE_START_POWER;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


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


    public Intake(HardwareMap hardwareMap) { init(hardwareMap); }

    //initialize intake
    private void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        intakeRunningContinuously = false;
        graduallyChangePower = false;

    }

    public boolean isIntakeRunningContinuously() { return intakeRunningContinuously; }

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

    //eq: a * BASE^(POW*time) + c = power
    //eq solution: (log[(power - c) / a]) / (log(b) * POW) = time
    private void graduallyChangePower(double targetPower) {
        intakeTargetPower = targetPower;
        intakeStartPower = INTAKE_START_POWER; //intake.getPower();
        graduallyChangePower = true;

        targetTime = (intakeTargetPower - intakeStartPower) / 1.0;



        /*
        //control for setting the function to get to targetPower
        if (intakeTargetPower == 0) {
            intake.setPower(0.0);
            graduallyChangePower = false;

        } else if (intakeTargetPower > intakeStartPower) {
            if (intakeStartPower > 0) {
                a = intakeStartPower;
                c = 0;

            } else if (intakeStartPower < 0) {
                a = -intakeStartPower;
                c = -2 * intakeStartPower;

            } else {
                intake.setPower(0.01);
                a = 0.01;
                c = 0.0;

            }

        } else if (intakeTargetPower < intakeStartPower) {
            if (intakeStartPower > 0) {
                a = -intakeStartPower;
                c = 2 * intakeStartPower;

            } else if (intakeStartPower < 0) {
                a = intakeStartPower;
                c = -2 * intakeStartPower;

            } else {
                intake.setPower(-0.01);
                a = -0.01;
                c = 0.0;

            }

        } else {
            //targetPower = currentPower
            graduallyChangePower = false;

        }

        //this is how long it should take to get to the targetPower
        targetTime = Math.log((targetPower - c) / a) / (INTAKE_GRADUAL_POW * Math.log(INTAKE_GRADUAL_BASE));
*/
        timer.reset();

    }

    public void updateIntake() {
        if (graduallyChangePower) {
            double time = timer.seconds();

            if (time >= targetTime) {
                intake.setPower(intakeTargetPower);
                graduallyChangePower = false;

            } else {
                intake.setPower(intakeStartPower + 1.0*time);
                //intake.setPower(Math.round((a * Math.pow(INTAKE_GRADUAL_BASE, INTAKE_GRADUAL_POW * time) + c) * 100.0) / 100.0);

            }

        }

    }

}