/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.CANifierControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

/**
 * Subsystem for controlling the turret
 */
public class Turret extends SubsystemBase {
    private final DriveTrain m_driveTrain;
    private final CANCoder encoder = new CANCoder(Constants.Turret.turretEncoder);
    private final VictorSPX turretMotor = new VictorSPX(Constants.Turret.turretMotor);
    private final DigitalInput turretHomeSensor = new DigitalInput(Constants.Turret.turretHomeSensor);
    
    private double setpoint = 0; //angle
    private boolean initialHome;
    private Constants.Turret.ControlMode controlMode = frc.robot.Constants.Turret.ControlMode.CLOSED_LOOP_UNSET;
    private boolean turretHomeSensorLatch = false;

    /**
     * Creates a new ExampleSubsystem.
     */
    public Turret(DriveTrain driveTrain) {
        // Setup turrent motors
        m_driveTrain = driveTrain;
        encoder.configFactoryDefault();
        encoder.setPositionToAbsolute();
        encoder.configSensorDirection(true);

        turretMotor.configFactoryDefault();
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.setInverted(true);
        turretMotor.configRemoteFeedbackFilter(61, RemoteSensorSource.CANCoder, 0, 0);
        turretMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        turretMotor.config_kF(0, Constants.Turret.kF);
        turretMotor.config_kP(0, Constants.Turret.kP);
        turretMotor.config_kI(0, Constants.Turret.kI);
        turretMotor.config_IntegralZone(0, Constants.Turret.kI_Zone);
        turretMotor.configMaxIntegralAccumulator(0, Constants.Turret.kMaxIAccum);
        turretMotor.config_kD(0, Constants.Turret.kD);
        turretMotor.configMotionCruiseVelocity(Constants.Turret.kCruiseVelocity);
        turretMotor.configMotionAcceleration(Constants.Turret.kMotionAcceleration);
        turretMotor.configAllowableClosedloopError(0, Constants.Turret.kErrorBand);

        //turretPID.enableContinuousInput(0, 360);

        //initShuffleboard();
    }

    // self-explanatory comnmands

    public void resetEncoder() {
        turretMotor.setSelectedSensorPosition(0);
        encoder.setPosition(0);
    }

    public Constants.Turret.ControlMode getControlMode() {
        return controlMode;
    }

    public void setControlMode(Constants.Turret.ControlMode mode) {
        controlMode = mode;
    }

    public double getTurretAngle() {
        return encoderUnitsToDegrees(turretMotor.getSelectedSensorPosition());
    }

    public double getFieldRelativeAngle() {
        return getTurretAngle() - m_driveTrain.getAngle();
    }

    public double getMaxAngle() {
        return Constants.Turret.maxAngle;
    }

    public double getMinAngle() {
        return Constants.Turret.minAngle;
    }

    public boolean getTurretHome() {
        return ! turretHomeSensor.get();
    }

    public boolean getInitialHome() { //Checks if the bot is in its starting position??
        return initialHome;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setPercentOutput(double output) {
        turretMotor.set(ControlMode.PercentOutput, output);
    }

    // ???
    public void setRobotCentricSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    // ???
    public void setFieldCentricSetpoint(double setpoint) {
        setpoint -= m_driveTrain.getAngle();

        if(setpoint > getMaxAngle())
            setpoint -= 360;
        else if(setpoint < getMinAngle())
            setpoint += 360;

        this.setpoint = setpoint;
    }

    public void setClosedLoopPosition() {
        turretMotor.set(ControlMode.MotionMagic, degreesToEncoderUnits(getSetpoint()));
    }

    public int degreesToEncoderUnits(double degrees) {
        return (int) (degrees * (1.0 / Constants.Turret.gearRatio) * (Constants.Turret.encoderUnitsPerRotation / 360.0));
    }

    public double encoderUnitsToDegrees(double encoderUnits) {
        return encoderUnits * Constants.Turret.gearRatio * (360.0 / Constants.Turret.encoderUnitsPerRotation);
    }

    // checks if the turret is pointing within the tolerance of the target
    public boolean onTarget() {
        return Math.abs(turretMotor.getClosedLoopError()) < Constants.Turret.kErrorBand;
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
        setpoint = getTurretAngle();
    }

    private void initShuffleboard() {
        // Unstable. Don''t use until WPILib fixes this
        Shuffleboard.getTab("Turret").addNumber("Turret Motor Output", turretMotor :: getMotorOutputPercent);
        Shuffleboard.getTab("Turret").addNumber("Turret Robot Relative Angle", this :: getTurretAngle);
        Shuffleboard.getTab("Turret").addNumber("Turret Field Relative Angle", this :: getFieldRelativeAngle);
        Shuffleboard.getTab("Turret").addNumber("Turret Setpoint", this :: getSetpoint);
        Shuffleboard.getTab("Turret").addNumber("Turret Error", turretMotor :: getClosedLoopError);
        Shuffleboard.getTab("Turret").addNumber("Turret IAccum", turretMotor :: getIntegralAccumulator);
        Shuffleboard.getTab("Turret").addBoolean("Home", this :: getTurretHome);
    }

    // set smartdashboard
    private void updateSmartdashboard() {
        if (RobotBase.isReal()) {
            SmartDashboard.putNumber("Turret Angle", getFieldRelativeAngle());

            SmartDashboard.putNumber(/*"Turret",*/ "Turret Motor Output", turretMotor.getMotorOutputPercent());
            SmartDashboard.putNumber(/*"Turret",*/ "Turret Robot Relative Angle", getTurretAngle());
            SmartDashboard.putNumber(/*"Turret",*/ "Turret Field Relative Angle", getFieldRelativeAngle());
            SmartDashboard.putNumber(/*"Turret",*/ "Turret Setpoint", getSetpoint());
            SmartDashboard.putBoolean(/*"Turret",*/ "Home", getTurretHome());
        }
    }

    @Override
    public void periodic() {
        if(getControlMode() == Constants.Turret.ControlMode.CLOSED_LOOP_SET)
            setClosedLoopPosition();

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

    public double getTurretSimAngle(){
        return getTurretAngle() + 180;
    }

    public Pose2d getTurretSimPose() {
        return new Pose2d(m_driveTrain.getRobotPose().getX(),
                          m_driveTrain.getRobotPose().getY(),
                          new Rotation2d(Math.toRadians(getTurretSimAngle())));
    }


    @Override
    public void simulationPeriodic() {
    }

    public double getIdealTargetDistance() {
        return Math.sqrt(Math.pow(Constants.SimConstants.blueGoalPose.getY() - getTurretSimPose().getY(), 2) + Math.pow(Constants.SimConstants.blueGoalPose.getX() - getTurretSimPose().getX(), 2));
    }

    public double getIdealTurretAngle() {

        double targetRadians = Math.atan2(Constants.SimConstants.blueGoalPose.getY() - getTurretSimPose().getY(), Constants.SimConstants.blueGoalPose.getX() - getTurretSimPose().getX());

        return Math.toDegrees(targetRadians);
    }
}
