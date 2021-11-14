package frc.robot;

public final class Constants {
    public static final class Indexer {
        public static final double kI_Zone = 1;
        public static final double maxVel = 1.1e4;
        public static final double maxAccel = 1e6;
        public static final double gearRatio = 1.0 / 27.0;

        // PID terms/other constants
        public static final double kF = 0.0001;
        public static final double kP = 0.000001;
        public static final double kI = 80;
        public static final double kD = 0.0001;
        
        public static final int indexerMotor = 35;
        public static final int kickerMotor = 36;
        public static final int intakeSensor = 0;
        public static final int indexerTopSensor = 1;
        public static final int indexerBottomSensor = 2;
    }

    public static final class Intake {        
        // PID and FeedForward loop terms
        public final double kFF = 0.00068; //0.06; //0.122
        public final double kP = 6e-5; //0.492
        public final double kI = 0;
        public final double kD = 0;

        public final double kI_Zone = 0;
        public final double allowableError = 50;
        public final double maxVel = 5880;
        public final double maxAccel = 58800;
        public final double gearRatio = 1.0 / 3.0;

        public static final int intakeMotor = 30;
        public static final int intakePistonForward = 2;
        public static final int intakePistonReverse = 3;

        public enum IntakeStates {
            INTAKE_EMPTY, INTAKE_ONE_BALL, INTAKE_FOUR_BALLS, INTAKE_FIVE_BALLS
        }
    }
    
    public static final int pcmOne = 11;
}
