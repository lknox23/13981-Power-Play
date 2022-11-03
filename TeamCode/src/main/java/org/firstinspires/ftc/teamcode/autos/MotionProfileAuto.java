package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.controllers.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.shplib.controllers.VelocityPID;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;

@Autonomous
public class MotionProfileAuto extends BaseRobot {
    double time;
    TrapezoidProfile profile;
    VelocityPID follower;
    SHPMotor motor;
    @Override
    public void start() {
        time = Clock.now();
        profile = new TrapezoidProfile(100);
        follower = new VelocityPID(Constants.Drive.kDriveP, motor.getVelocity(MotorUnit.TICKS));
        motor = new SHPMotor(hardwareMap, "Testing motor");
    }

    @Override
    public void loop() {
        motor.setPower(follower.calculate(profile.getVelocity(Clock.elapsed(time))));
    }
}
