package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class MoveClawCommand extends Command {
    private final ClawSubsystem claw;
    private ElapsedTime time;

    public enum Direction {
        OPEN,
        CLOSED
    }

    Direction direction;

    public MoveClawCommand(ClawSubsystem claw, Direction direction) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(claw);

        this.claw = claw;
        this.direction = direction;
    }

    // Called once when the command is initially schedule
    @Override
    public void init() {
        if (direction == Direction.OPEN)
            claw.setState(ClawSubsystem.State.OPEN);
        else
            claw.setState(ClawSubsystem.State.CLOSED);
        time = new ElapsedTime();
        time.reset();
    }

    // Called repeatedly until isFinished() returns true
    @Override
    public void execute() {

    }

    // Called once after isFinished() returns true
    @Override
    public void end() {

    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return time.seconds() > 0.5;
    }
}
