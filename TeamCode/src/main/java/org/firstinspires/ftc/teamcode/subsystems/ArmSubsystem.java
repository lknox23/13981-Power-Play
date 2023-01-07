package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit.TICKS;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;

// Use this class as a reference for creating new subsystems

public class ArmSubsystem extends Subsystem {
    public final SHPMotor slide;
    //public final SHPMotor actuator;
    public enum State {
        TOP,
        MIDDLE,
        BOTTOM,
    }

    private State state;
    public boolean manual;
    private double manualPosition;
    private double previousTime;

    public ArmSubsystem(HardwareMap hardwareMap) {
        slide = new SHPMotor(hardwareMap, Constants.Arm.kSlideName, TICKS);
        slide.enablePositionPID(Constants.Arm.kSlideP);
        slide.setPositionErrorTolerance(Constants.Arm.kSlideTolerance);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
//        slide.enableVelocityPID(Constants.Arm.kSlideP);
//        slide.enableProfiling(Constants.Arm.kSlideMaxVelocity);
/*

        actuator = new SHPMotor(hardwareMap, Constants.Arm.kActuatorName);
        actuator.enablePositionPID(Constants.Arm.kActuatorP);
        actuator.setPositionErrorTolerance(Constants.Arm.kActuatorTolerance);
*/
        previousTime = Clock.now();
        slide.resetEncoder();
        setState(State.BOTTOM);
    }

    public void setState(State state) {
        this.state = state;
        manual = false;
        previousTime = Clock.now();
    }

    public void setManual(double manualIncrement) {
        manualPosition = slide.getPosition(TICKS) + manualIncrement;
        if (manualPosition<Constants.Arm.kSlideBottom)
            manualPosition = Constants.Arm.kSlideBottom;
        else if (manualPosition> Constants.Arm.kSlideTop)
            manualPosition = Constants.Arm.kSlideTop;
        manual = true;
    }

    public void nextState() {
        if (this.state == State.MIDDLE) setState(State.TOP);
        else if (this.state == State.BOTTOM) setState(State.MIDDLE);
        manual = false;
    }

    public void previousState() {
        if (this.state == State.TOP) setState(State.MIDDLE);
        else if (this.state == State.MIDDLE) setState(State.BOTTOM);
        manual = false;
    }

    public boolean atSetpoint() {
        return slide.atPositionSetpoint();
    }

    public boolean atBottom() {
        return this.state == State.BOTTOM;
    }

    @Override
    public void periodic(Telemetry telemetry) {
        telemetry.addData("slide ticks: ", slide.getPosition(TICKS));
        //telemetry.addData("actuator enc: ", actuator.getPosition(MotorUnit.TICKS));
        telemetry.addData("slide power: ", slide.getPower());
        telemetry.addData("arm at setpoint: ", atSetpoint() ? "true" : "false");
        telemetry.addData("time: ", Clock.elapsed(previousTime));
//        telemetry.addData("profile output: ", slide.followProfile(Clock.elapsed(previousTime)));
        if (manual) {
            slide.setPosition(manualPosition);
            telemetry.addData("manual: ", "TRUE");
            return;
        } else
            telemetry.addData("manual: ", "FALSE");
        switch (state) {
            case TOP:
                slide.setPosition(Constants.Arm.kSlideTop);
                //actuator.setPosition(Constants.Arm.kActuatorTop);
                telemetry.addData("state: ", "TOP");
                break;
            case MIDDLE:
                slide.setPosition(Constants.Arm.kSlideMiddle);
                //actuator.setPosition(Constants.Arm.kActuatorMiddle);
                telemetry.addData("state: ", "MIDDLE");
                break;
            case BOTTOM:
                slide.setPosition(Constants.Arm.kSlideBottom);
                //actuator.setPosition(Constants.Arm.kActuatorBottom);
                telemetry.addData("state: ", "BOTTOM");
                break;
        }
    }
}
