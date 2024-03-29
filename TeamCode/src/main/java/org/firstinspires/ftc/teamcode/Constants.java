package org.firstinspires.ftc.teamcode;

public final class Constants {
    // Target voltage for voltage compensation
    public static final double kNominalVoltage = 12.0;

    // Multiplied by each term before assigning to the controller
    public static final double kPositionPIDFactor = 1.0 / 10000.0;
    public static final double kVelocityPIDFactor = 1.0 / 1000.0;

    public static final class Drive {
        public static final String[] kMotorNames = new String[]{
                "leftFront",
                "leftRear",
                "rightFront",
                "rightRear"
        };

        public static final double kDriveP = 5;

        //ticks/s^
        public static final double vMax = 30;
        //ticks/s
        public static final double aMax = 30;
    }

    public static final class Vision {
        public static final double kTagsizeMeters = 0.0475;
    }

    public static final class Arm {
        public static final String kSlideName = "slide";
        public static final String kActuatorName = "actuator";

        public static final double kSlideP = 30; //50
        public static final double kSlideTolerance = 80;
        public static final double kSlideMaxVelocity = 1;

        public static final double kActuatorP = 5;
        public static final double kActuatorTolerance = 70;

        public static final double kSlideTop = 4300;
        public static final double kSlideMiddle = 2000;
        public static final double kSlideBottom = 50;

        public static final double kManualIncrement = 300;
    }

    public static final class Claw {
        public static final String kClawName = "claw";

        public static final double kOpen = 0;
        public static final double kClosed = 0.3;
    }
}
