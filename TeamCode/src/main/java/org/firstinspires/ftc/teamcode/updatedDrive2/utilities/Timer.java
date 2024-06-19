package org.firstinspires.ftc.teamcode.updatedDrive2.utilities;


import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

/**
 * Stores all the the time sequences
 */

public class Timer {
    private ElapsedTime internalTimer;
    private double currentTime;
    private ArrayList<IndividualTimer> timerList;
    private ArrayList<String> returnList;

    public Timer() {
        internalTimer = new ElapsedTime();

    }

    public void addTimer(String name, double seconds) {
        timerList.add(new IndividualTimer(name, internalTimer.seconds() + seconds));

    }

    public ArrayList<String> update() {
        currentTime = internalTimer.seconds();
        returnList.clear();

        for (IndividualTimer timer : timerList) {
            if (timer.getTargetTime() >= currentTime) {
                returnList.add(timer.getTimerName());

            }

        }

        return returnList;

    }


}