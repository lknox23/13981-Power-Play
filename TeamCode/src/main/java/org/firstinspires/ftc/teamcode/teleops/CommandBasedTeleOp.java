package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem.State.CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem.State.OPEN;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.commands.MoveClawCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

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
        drive.mecanum(-gamepad1.left_stick_y*0.5, -gamepad1.left_stick_x*0.5, gamepad1.right_stick_x*0.5);
        new Trigger(gamepad1.dpad_up, new RunCommand(()->{arm.setManual(300);}));
        new Trigger(gamepad1.dpad_down, new RunCommand(()->{arm.setManual(-300);}));
        /*
        new Trigger(gamepad1.a && armDown,
                new RunCommand(()-> {
                    claw.setState(ClawSubsystem.State.CLOSED);
                    armDown = false;})
                        .then(new WaitCommand(0.5))
                        .then(new MoveArmCommand(arm, MoveArmCommand.Direction.TOP))
        );
        new Trigger(gamepad1.a && !armDown, new ParallelCommandGroup(
                new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM),
                new WaitCommand(0.5).then
                        (new RunCommand(()->{claw.setState(OPEN);})),
                new RunCommand(()->{armDown = true;})
        ));

         */
        /*
        new Trigger(gamepad1.a && arm.atBottom(),
                new MoveClawCommand(claw, MoveClawCommand.Direction.CLOSED).then(
                new MoveArmCommand(arm, MoveArmCommand.Direction.TOP))
        );

         */
        /*
        new Trigger(gamepad1.a && !arm.atBottom(),
                new MoveClawCommand(claw, MoveClawCommand.Direction.OPEN).then(
                        new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM)
                )
//                new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM).with(
//                        new WaitCommand(0.5).then(
//                                new MoveClawCommand(claw, MoveClawCommand.Direction.OPEN)
//                        )
//                )
//
//                new ParallelCommandGroup(
//                        new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM),
//                        new SequentialCommandGroup(
//                                new WaitCommand(0.5),
//                                new MoveClawCommand(claw, MoveClawCommand.Direction.OPEN)
//                        )
//
//                )

        );
         */
        new Trigger(gamepad1.a, new MoveArmCommand(arm, MoveArmCommand.Direction.TOP));
        new Trigger(gamepad1.b, new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM));
        new Trigger(gamepad1.left_bumper, new MoveClawCommand(claw, MoveClawCommand.Direction.OPEN));
        new Trigger(gamepad1.right_bumper, new MoveClawCommand(claw, MoveClawCommand.Direction.CLOSED));

        new Trigger(gamepad1.start, new RunCommand(()->drive.resetOrientation()));
        telemetry.addData("arm at bottom: ", arm.atBottom());
        telemetry.update();
    }
}
