package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class SampleTeleOp extends OpMode {
    DcMotor leftBack;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor rightBack;

    @Override
    public void init() {
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
//        double forward = gamepad1.left_stick_y;
//        double turn = gamepad1.left_stick_x;
//
//
//        double leftPower = Range.clip(forward + turn, -1.0, 1.0);
//        double rightPower = Range.clip(forward - turn, -1.0, 1.0);
//        leftBack.setPower(leftPower);
//        leftFront.setPower(leftPower);
//        rightBack.setPower(rightPower);
//        rightFront.setPower(rightPower);

        leftBack.setPower(1);
        leftFront.setPower(1);
        rightBack.setPower(1);
        rightFront.setPower(1);

        telemetry.addData("left back ", leftBack.getPower());
        telemetry.addData("left front ", leftFront.getPower());
        telemetry.addData("right back ", rightBack.getPower());
        telemetry.addData("right front ", rightFront.getPower());
    }
}
