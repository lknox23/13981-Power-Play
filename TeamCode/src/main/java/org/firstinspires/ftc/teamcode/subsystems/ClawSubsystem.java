package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class ClawSubsystem extends Subsystem {
    public final Servo claw;

    public enum State {
        OPEN,
        CLOSED
    }

    private State state;

    public ClawSubsystem(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, Constants.Claw.kClawName);

        this.state = State.CLOSED;
    }

    public void setState(State state) {
        this.state = state;
        switch (state) {
            case OPEN:
                claw.setPosition(Constants.Claw.kOpen);
                break;
            case CLOSED:
                claw.setPosition(Constants.Claw.kClosed);
                break;
        }
    }

    @Override
    public void periodic(Telemetry telemetry) {
        telemetry.addData("servo position: ", claw.getPosition());
    }
}
