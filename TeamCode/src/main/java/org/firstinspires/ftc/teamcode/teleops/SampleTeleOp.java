package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.commands.DumpCargoCommand;
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.subsystems.ScoopSubsystem;

@TeleOp
public class SampleTeleOp extends OpMode {
    DcMotor leftBack;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor rightBack;

    @Override
    public void init() {
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        double leftY = gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;

        double leftPower = Range.clip(leftY + leftX, -1.0, 1.0);
        double rightPower = Range.clip(leftY - leftX, -1.0, 1.0);
        leftBack.setPower(leftPower);
        leftFront.setPower(leftPower);
        rightBack.setPower(rightPower);
        rightFront.setPower(rightPower);


    }
}
