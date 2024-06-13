package org.firstinspires.ftc.teamcode.updatedDrive.utilities;

public class IndividualTimer {
    private double timerLength, targetTime;
    private String timerName;

    public IndividualTimer(String nameOfTimer, double targetOfTimer) {
        targetTime = targetOfTimer;
        timerName = nameOfTimer;

    }

    public double getTargetTime() { return targetTime; }

    public String getTimerName() { return timerName; }

}