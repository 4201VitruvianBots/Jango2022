package frc.robot;

public class Constants {
    public class Indexer {
        public static final int indexerMotor = 35;
        public static final int kickerMotor = 36;
        public static final int intakeSensor = 0;
        public static final int indexerTopSensor = 1;
        public static final int indexerBottomSensor = 2;
    }

    public class Intake {        
        // PID and FeedForward loop terms
        private final double kFF = 0.00068; //0.06; //0.122
        private final double kP = 6e-5; //0.492
        private final double kI = 0;
        private final double kD = 0;

        private final double kI_Zone = 0;
        private final double allowableError = 50;
        private final double maxVel = 5880;
        private final double maxAccel = 58800;
        private final double gearRatio = 1.0 / 3.0;

        public static final int intakeMotor = 30;
        public static final int pcmOne = 11;
        public static final int intakePistonForward = 2;
        public static final int intakePistonReverse = 3;
    }
}
