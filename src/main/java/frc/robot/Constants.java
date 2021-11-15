package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public static class Turret {    
        public static final int turretMotor = 60;
        public static final int turretEncoder = 61;

        public static final int turretHomeSensor = 3;

        public static final double turretAccelerationRadiansPerSecond = .75;

        public static final int encoderUnitsPerRotation = 4096;
    
        // Turret PID gains
        public static final double kF = 0.07;
        public static final double kP = 0.2;
        public static final double kI = 0.0015;
        public static final double kD = 0.0;
        public static final int kI_Zone = 900;
        public static final int kMaxIAccum = 1000000;
        public static final int kErrorBand = 50;
        public static final int kCruiseVelocity = 10000;
        public static final int kMotionAcceleration = kCruiseVelocity * 10;
        public static final double minAngle = - 90;
        public static final double maxAngle = 90;
        public static final double gearRatio = 18.0 / 120.0;
        
        public static enum ControlMode {
            CLOSED_LOOP_SET, CLOSED_LOOP_UNSET
        }
    }

    public static class SimConstants {
        public static final Pose2d blueGoalPose = new Pose2d(0, 5.831, new Rotation2d());
    }
}
