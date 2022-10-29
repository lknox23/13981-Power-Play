package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.controllers.PositionPID;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TemplateSubsystem;

public class DriveForwardCommand extends Command {
    private final DriveSubsystem drive;
    public DriveForwardCommand(DriveSubsystem drive) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(drive);
        this.drive = drive;
    }

    // Called once when the command is initially schedule
    @Override
    public void init() {

    }

    // Called repeatedly until isFinished() returns true
    @Override
    public void execute() {
        drive.mecanum(1, 0, 0);
    }

    // Called once after isFinished() returns true
    @Override
    public void end() {

    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return true;
    }
}
