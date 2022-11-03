package org.firstinspires.ftc.teamcode.shplib.controllers;

public interface MotionProfile {
    /**
     *
     * @param time time elapsed since following began (seconds)
     * @return desired velocity (inches/second)
     */
    public double getVelocity(double time);
}
