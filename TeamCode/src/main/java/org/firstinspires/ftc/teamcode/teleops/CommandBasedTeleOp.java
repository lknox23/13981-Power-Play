package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.commands.DumpCargoCommand;
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.subsystems.ScoopSubsystem;

@TeleOp
public class CommandBasedTeleOp extends BaseRobot {

    @Override
    public void init() {
        super.init();

    }

    @Override
    public void start() {
        super.start();

        // Add anything that needs to be run a single time when the OpMode starts
    }

    @Override
    public void loop() {
        // Allows CommandScheduler.run() to be called - DO NOT DELETE!
        super.loop();
        drive.mecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

    }
}
