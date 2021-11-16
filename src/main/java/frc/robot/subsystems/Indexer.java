package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.CANConstants;

/**
 * Susbsystem for interacting with the robot's indexer (feeds balls from intake to shooter)
 */
public class Indexer extends SubsystemBase {

    // Setup indexer motor controller (SparkMax)
    CANSparkMax master = new CANSparkMax(CANConstants.indexerMotor, MotorType.kBrushless);
    CANEncoder encoder = master.getEncoder();
    CANPIDController pidController = master.getPIDController();
    VictorSPX kicker = new VictorSPX(CANConstants.kickerMotor);
    // Indexer sensors setup
    DigitalInput intakeSensor = new DigitalInput(CANConstants.intakeSensor);
    DigitalInput indexerTopSensor = new DigitalInput(CANConstants.indexerTopSensor);
    DigitalInput indexerBottomSensor = new DigitalInput(CANConstants.indexerBottomSensor);
    // Detect whether a new ball has been picked up
    // There is a new ball if the intake sensor is blocked and was not blocked before
    boolean pTripped = false;
    private double targetSetpoint;

    /**
     * Creates a new Indexer.
     */
    public Indexer() {
        // Motor and PID controller setup
        master.restoreFactoryDefaults();
        master.setInverted(false);

        master.setIdleMode(IdleMode.kBrake);

        pidController.setFF(IndexerConstants.kF);
        pidController.setP(IndexerConstants.kP);
        pidController.setI(IndexerConstants.kI);
        pidController.setD(IndexerConstants.kD);
        pidController.setSmartMotionMaxVelocity(IndexerConstants.maxVel, 0); // Formerly 1.1e4
        pidController.setSmartMotionMaxAccel(IndexerConstants.maxAccel, 0); // Formerly 1e6
        pidController.setSmartMotionAllowedClosedLoopError(1, 0);
        pidController.setIZone(IndexerConstants.kI_Zone);

        kicker.configFactoryDefault();
        kicker.setInverted(true);
    }

    public boolean getIntakeSensor() {
        return (! intakeSensor.get());
    }

    public boolean getIndexerBottomSensor() {
        return ! indexerBottomSensor.get();
    }

    public boolean getIndexerTopSensor() {
        return ! indexerTopSensor.get();
    }

    public void setKickerPercentOutput(double output) {
        kicker.set(ControlMode.PercentOutput, output);
    }

    /**
     * @param output Speed value from -1 to 1
     */
    public void setIndexerPercentOutput(double output) {
        master.set(output);
    }

    public boolean newBall() {
        boolean returnVal;
        returnVal = pTripped == false && getIntakeSensor();
        pTripped = getIntakeSensor();
        return returnVal;
    }

    public void setRPM(double rpm) {
        double setpoint = rpm / IndexerConstants.gearRatio;
        SmartDashboard.putNumber("Indexer Setpoint", setpoint);
        pidController.setReference(setpoint, ControlType.kSmartVelocity);
    }

    private void initShuffleboard() {
        // Unstable. Don''t use until WPILib fixes this
        Shuffleboard.getTab("Indexer").addBoolean("Intake Sensor", this :: getIntakeSensor);
        Shuffleboard.getTab("Indexer").addBoolean("Indexer Bottom Sensor", this :: getIndexerBottomSensor);
        Shuffleboard.getTab("Indexer").addBoolean("Indexer Top Sensor", this :: getIndexerTopSensor);
    }

    private void updateSmartDashboard() {
        // TODO: Make these SmartDashboardTab when the library is updated
        SmartDashboard.putBoolean(/*"Indexer",*/ "Intake Sensor", getIntakeSensor());
        SmartDashboard.putBoolean(/*"Indexer",*/ "Indexer Bottom Sensor", getIndexerBottomSensor());
        SmartDashboard.putBoolean(/*"Indexer",*/ "Indexer Top Sensor", getIndexerTopSensor());
    }

    private void updatePIDValues() {
        // Allow PID values to be set through SmartDashboard
        IndexerConstants.kF = SmartDashboard.getNumber("kF", 0);
        IndexerConstants.kP = SmartDashboard.getNumber("kP", 0);
        IndexerConstants.kI = SmartDashboard.getNumber("kI", 0);
        IndexerConstants.kD = SmartDashboard.getNumber("kD", 0);
        pidController.setFF(IndexerConstants.kF);
        pidController.setP(IndexerConstants.kP);
        pidController.setI(IndexerConstants.kI);
        pidController.setD(IndexerConstants.kD);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateSmartDashboard();
    }
}
