package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TemplateSubsystem;

public class ArmMacroCommand extends Command {
    private final ArmSubsystem arm;
    private final ClawSubsystem claw;
    private final Direction direction;
    private double startTime;
    //private double stackHeight = 300;
    //private final double stackIncrement = 50;

    public enum Direction {
        UP, DOWN, STACK
    }

    public ArmMacroCommand(ArmSubsystem arm, ClawSubsystem claw, Direction direction) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(arm, claw);

        this.arm = arm;
        this.claw = claw;
        this.direction = direction;
    }

    // Called once when the command is initially schedule
    @Override
    public void init() {
        startTime = Clock.now();
        if (direction == Direction.UP)
            claw.setState(ClawSubsystem.State.CLOSED);
        else
            claw.setState(ClawSubsystem.State.OPEN);
    }

    // Called repeatedly until isFinished() returns true
    @Override
    public void execute() {
        if (Clock.hasElapsed(startTime, 1)) {
            if (direction == Direction.UP)
                arm.setState(ArmSubsystem.State.TOP);
            else
                arm.setState(ArmSubsystem.State.BOTTOM);
        }
    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return arm.atSetpoint() && Clock.hasElapsed(startTime, 1);
    }
}
