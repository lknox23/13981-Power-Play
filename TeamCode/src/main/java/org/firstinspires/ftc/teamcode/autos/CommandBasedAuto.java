package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

@Autonomous
public class CommandBasedAuto extends BaseRobot {
    ArmSubsystem arm;
    ClawSubsystem claw;
    @Override
    public void init() {
        super.init();
        arm = new ArmSubsystem(hardwareMap);
        claw = new ClawSubsystem(hardwareMap);
    }

    @Override
    public void start() {
        super.start();
        CommandScheduler.getInstance().scheduleCommand(
                new MoveArmCommand(arm, MoveArmCommand.Direction.TOP)
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM))
        );

    }
}
