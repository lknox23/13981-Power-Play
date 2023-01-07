package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.openftc.apriltag.AprilTagDetection;

public class FindAprilTagCommand extends Command {
    private final VisionSubsystem vision;
    private double startTime;
    private double minTime = 0;

    public FindAprilTagCommand(VisionSubsystem vision) {
        // Pass through any subsystems that are uninterruptible
        super(vision);

        this.vision = vision;
    }
    public FindAprilTagCommand(VisionSubsystem vision, double minTime) {
        // Pass through any subsystems that are uninterruptible
        super(vision);

        this.vision = vision;
        this.minTime = minTime;
    }

    @Override
    public void init() {
        this.startTime = Clock.now();
        if (!vision.detectedTags())
            vision.setState(VisionSubsystem.State.ENABLED);
    }

    @Override
    public void execute() {
        if (vision.detectedTags())
            vision.setState(VisionSubsystem.State.DISABLED);
    }

    @Override
    public boolean isFinished() {
        return vision.detectedTags() && Clock.hasElapsed(startTime, minTime) || Clock.hasElapsed(startTime, minTime+5);
    }

    @Override
    public void end() {
        vision.setState(VisionSubsystem.State.DISABLED);
    }
}
