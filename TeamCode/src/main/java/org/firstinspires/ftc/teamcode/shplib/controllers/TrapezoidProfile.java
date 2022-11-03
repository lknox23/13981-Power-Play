package org.firstinspires.ftc.teamcode.shplib.controllers;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.rotationsToInches;

import org.firstinspires.ftc.teamcode.Constants;

public class TrapezoidProfile implements MotionProfile{
    private double distance;
    private double vMax;
    private double aMax;
    private double tTotal;
    private double tA;

    public TrapezoidProfile() {
        vMax = 0;
        aMax = 0;
        tTotal = 0;
        tA = 0;
    }

    /**
     * @param distance distance to be travelled, in ticks
     */
    public TrapezoidProfile(double distance) {
        this.vMax = Constants.Drive.vMax;
        this.aMax = Constants.Drive.aMax;
        this.tA = Math.sqrt(distance / aMax);
    }

    /**
     *
     * @param time time elapsed since path following started
     * @return desired velocity (ticks/second)
     */
    public double getVelocity(double time) {
        if (time < tA)
            return encoderTicksToInches(.5 * aMax * time);
        else if (time < tTotal - tA)
            return .5 * aMax * tA;
        else if (time < tTotal)
            return .5 * aMax * (tTotal - time);
        else
            return 0;
    }
}
