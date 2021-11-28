/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.DriveTrain;

/**
 * Subsystem for controlling the turret
 */
public class Turret extends SubsystemBase {
    private final DriveTrain m_driveTrain;
    private final CANCoder encoder = new CANCoder(CANConstants.turretEncoder);
    private final VictorSPX turretMotor = new VictorSPX(CANConstants.turretMotor);
    private final DigitalInput turretHomeSensor = new DigitalInput(CANConstants.turretHomeSensor);
    
    private double setpoint = 0; //angle
    private boolean initialHome;
    private boolean usingSensor = false;
    private boolean turretHomeSensorLatch = false;

    /**
     * Creates a new Turret.
     */
    public Turret(DriveTrain driveTrain) {
        // Setup turret motors
        m_driveTrain = driveTrain;
        encoder.configFactoryDefault();
        encoder.setPositionToAbsolute();
        encoder.configSensorDirection(true);

        turretMotor.configFactoryDefault();
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.setInverted(true);
        turretMotor.configRemoteFeedbackFilter(61, RemoteSensorSource.CANCoder, 0, 0);
        turretMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        turretMotor.config_kF(0, TurretConstants.kF);
        turretMotor.config_kP(0, TurretConstants.kP);
        turretMotor.config_kI(0, TurretConstants.kI);
        turretMotor.config_IntegralZone(0, TurretConstants.kI_Zone);
        turretMotor.configMaxIntegralAccumulator(0, TurretConstants.kMaxIAccum);
        turretMotor.config_kD(0, TurretConstants.kD);
        turretMotor.configMotionCruiseVelocity(TurretConstants.kCruiseVelocity);
        turretMotor.configMotionAcceleration(TurretConstants.kMotionAcceleration);
        turretMotor.configAllowableClosedloopError(0, TurretConstants.kErrorBand);
    }

    public void resetEncoder() {
        turretMotor.setSelectedSensorPosition(0);
        encoder.setPosition(0);
    }

    public boolean getUsingSensor() {
        return usingSensor;
    }

    public void setUsingSensor(boolean using) {
        usingSensor = using;
    }

    public double getTurretAngleDegrees() {
        return encoderUnitsToDegrees(turretMotor.getSelectedSensorPosition());
    }

    public double getFieldRelativeAngleDegrees() {
        return getTurretAngleDegrees() - m_driveTrain.getHeadingDegrees();
    }

    public double getMaxAngleDegrees() {
        return TurretConstants.maxAngleDegrees;
    }

    public double getMinAngleDegrees() {
        return TurretConstants.minAngleDegrees;
    }
    public boolean getTurretHome() {
        return !turretHomeSensor.get();
    }

    /**
     * Checks if the robot is in its starting position
     */
    public boolean getInitialHome() {
        return initialHome;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setPercentOutput(double output) {
        turretMotor.set(ControlMode.PercentOutput, output);
    }

    public void setRobotCentricSetpointDegrees(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setFieldCentricSetpointDegrees(double setpoint) {
        setpoint -= m_driveTrain.getHeadingDegrees();

        if(setpoint > getMaxAngleDegrees())
            setpoint -= 360;
        else if(setpoint < getMinAngleDegrees())
            setpoint += 360;

        this.setpoint = setpoint;
    }

    public void setClosedLoopPositionDegrees() {
        turretMotor.set(ControlMode.MotionMagic, degreesToEncoderUnits(getSetpoint()));
    }

    public int degreesToEncoderUnits(double degrees) {
        return (int) (degrees * (1.0 / TurretConstants.gearRatio) * (TurretConstants.encoderUnitsPerRotation / 360.0));
    }

    public double encoderUnitsToDegrees(double encoderUnits) {
        return encoderUnits * TurretConstants.gearRatio * (360.0 / TurretConstants.encoderUnitsPerRotation);
    }

    /**
     * checks if the turret is pointing within the tolerance of the target
     */
    public boolean onTarget() {
        return Math.abs(turretMotor.getClosedLoopError()) < TurretConstants.kErrorBand;
    }

    public void clearIAccum() {
        turretMotor.setIntegralAccumulator(0);
    }

    private void setTurretLatch(boolean state) {
        turretHomeSensorLatch = state;
    }

    public boolean getTurretLatch() {
        return turretHomeSensorLatch;
    }

    public void stopTurret() {
        setpoint = getTurretAngleDegrees();
    }

    private void initShuffleboard() {
        // Unstable. Don''t use until WPILib fixes this
        Shuffleboard.getTab("Turret").addNumber("Turret Motor Output", turretMotor :: getMotorOutputPercent);
        Shuffleboard.getTab("Turret").addNumber("Turret Robot Relative Angle", this :: getTurretAngleDegrees);
        Shuffleboard.getTab("Turret").addNumber("Turret Field Relative Angle", this :: getFieldRelativeAngleDegrees);
        Shuffleboard.getTab("Turret").addNumber("Turret Setpoint", this :: getSetpoint);
        Shuffleboard.getTab("Turret").addNumber("Turret Error", turretMotor :: getClosedLoopError);
        Shuffleboard.getTab("Turret").addNumber("Turret IAccum", turretMotor :: getIntegralAccumulator);
        Shuffleboard.getTab("Turret").addBoolean("Home", this :: getTurretHome);
    }

    // set smartdashboard
    private void updateSmartdashboard() {
        if (RobotBase.isReal()) {
            SmartDashboard.putNumber("Turret Angle", getFieldRelativeAngleDegrees());

            SmartDashboard.putNumber(/*"Turret",*/ "Turret Motor Output", turretMotor.getMotorOutputPercent());
            SmartDashboard.putNumber(/*"Turret",*/ "Turret Robot Relative Angle", getTurretAngleDegrees());
            SmartDashboard.putNumber(/*"Turret",*/ "Turret Field Relative Angle", getFieldRelativeAngleDegrees());
            SmartDashboard.putNumber(/*"Turret",*/ "Turret Setpoint", getSetpoint());
            SmartDashboard.putBoolean(/*"Turret",*/ "Home", getTurretHome());
        }
    }

    @Override
    public void periodic() {
        if(getUsingSensor())
            setClosedLoopPositionDegrees();

        // This method will be called once per scheduler run
        // TODO: FIX
        // Fix what??
        if(! getTurretLatch() && getTurretHome()) {
            turretMotor.setSelectedSensorPosition(0);
            encoder.setPosition(0);
            setTurretLatch(true);
        } else if(getTurretLatch() && ! getTurretHome())
            setTurretLatch(false);

        if(! initialHome)
            if(getTurretHome())
                initialHome = true;

        updateSmartdashboard();
    }

    public double getTurretSimAngleDegrees() {
        return getTurretAngleDegrees() + 180;
    }

    public Pose2d getTurretSimPoseMeters() {
        return new Pose2d(m_driveTrain.getRobotPoseMeters().getX(),
                          m_driveTrain.getRobotPoseMeters().getY(),
                          new Rotation2d(Math.toRadians(getTurretSimAngleDegrees())));
    }


    @Override
    public void simulationPeriodic() {
    }

    public double getIdealTargetDistanceMeters() {
        return Math.sqrt(Math.pow(SimConstants.blueGoalPoseMeters.getY() - getTurretSimPoseMeters().getY(), 2) + Math.pow(SimConstants.blueGoalPoseMeters.getX() - getTurretSimPoseMeters().getX(), 2));
    }

    public double getIdealTurretAngle() {

        double targetRadians = Math.atan2(SimConstants.blueGoalPoseMeters.getY() - getTurretSimPoseMeters().getY(), SimConstants.blueGoalPoseMeters.getX() - getTurretSimPoseMeters().getX());

        return Math.toDegrees(targetRadians);
    }
}
