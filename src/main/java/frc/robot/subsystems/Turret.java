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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.simulation.SimConstants;

/*
Subsystem for controlling the turret
 */

public class Turret extends SubsystemBase {
    private final int encoderUnitsPerRotation = 4096;
    private final DriveTrain m_driveTrain;
    private final Timer timeout = new Timer();
    private final CANCoder encoder = new CANCoder(CANConstants.turretEncoder);
    private final VictorSPX turretMotor = new VictorSPX(CANConstants.turretMotor);
    private final DigitalInput turretHomeSensor = new DigitalInput(TurretConstants.turretHomeSensor);

    // setup variables

    // Turret PID gains
    double kF = 0.07;
    double kP = 0.2;
    double kI = 0.0015;
    double kD = 0.0;
    int kI_Zone = 900;
    int kMaxIAccum = 1000000;
    int kErrorBand = 50;
    int kCruiseVelocity = 10000;
    int kMotionAcceleration = kCruiseVelocity * 10;
    double minAngleDegrees = - 90;
    double maxAngleDegrees = 90;
    double gearRatio = 18.0 / 120.0;
    private double setpoint = 0; //angle
    private int controlMode = 1;
    private boolean initialHome;
    private boolean turretHomeSensorLatch = false;

    public Turret(DriveTrain driveTrain) {
        m_driveTrain = driveTrain;
        encoder.configFactoryDefault();
        encoder.setPositionToAbsolute();
        encoder.configSensorDirection(true);

        turretMotor.configFactoryDefault();
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.setInverted(true);
        turretMotor.configRemoteFeedbackFilter(61, RemoteSensorSource.CANCoder, 0, 0);
        turretMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        turretMotor.config_kF(0, kF);
        turretMotor.config_kP(0, kP);
        turretMotor.config_kI(0, kI);
        turretMotor.config_IntegralZone(0, kI_Zone);
        turretMotor.configMaxIntegralAccumulator(0, kMaxIAccum);
        turretMotor.config_kD(0, kD);
        turretMotor.configMotionCruiseVelocity(kCruiseVelocity);
        turretMotor.configMotionAcceleration(kMotionAcceleration);
        turretMotor.configAllowableClosedloopError(0, kErrorBand);

        //initShuffleboard();
    }

    public void resetEncoder() {
        turretMotor.setSelectedSensorPosition(0);
        encoder.setPosition(0);
    }

    public int getControlMode() {
        return controlMode;
    }

    public void setControlMode(int mode) {
        controlMode = mode;
    }

    public double getTurretAngleDegrees() {
        return encoderUnitsToDegrees(turretMotor.getSelectedSensorPosition());
    }

    public double getFieldRelativeAngleDegrees() {
        return getTurretAngleDegrees() - m_driveTrain.getHeadingDegrees();
    }

    public double getMaxAngleDegrees() {
        return maxAngleDegrees;
    }

    public double getMinAngleDegrees() {
        return minAngleDegrees;
    }

    public boolean getTurretHome() {
        return ! turretHomeSensor.get();
    }

    public boolean getInitialHome() { //Checks if the robot is in its starting position
        return initialHome;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setPercentOutput(double output) {
        turretMotor.set(ControlMode.PercentOutput, output);
    }

    public void setRobotCentricSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setFieldCentricSetpoint(double setpoint) {
        setpoint -= m_driveTrain.getHeadingDegrees();

        if(setpoint > getMaxAngleDegrees())
            setpoint -= 360;
        else if(setpoint < getMinAngleDegrees())
            setpoint += 360;

        this.setpoint = setpoint;
    }

    public void setClosedLoopPosition() {
        turretMotor.set(ControlMode.MotionMagic, degreesToEncoderUnits(getSetpoint()));
    }

    public int degreesToEncoderUnits(double degrees) {
        return (int) (degrees * (1.0 / gearRatio) * (encoderUnitsPerRotation / 360.0));
    }

    public double encoderUnitsToDegrees(double encoderUnits) {
        return encoderUnits * gearRatio * (360.0 / encoderUnitsPerRotation);
    }

    // checks if the turret is pointing within the tolerance of the target
    public boolean onTarget() {
        return Math.abs(turretMotor.getClosedLoopError()) < kErrorBand;
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

    private void updateSmartdashboard() {
        if (RobotBase.isReal()) {
            SmartDashboard.putNumber("Turret Angle", getFieldRelativeAngleDegrees());

            SmartDashboardTab.putNumber("Turret", "Turret Motor Output", turretMotor.getMotorOutputPercent());
            SmartDashboardTab.putNumber("Turret", "Turret Robot Relative Angle", getTurretAngleDegrees());
            SmartDashboardTab.putNumber("Turret", "Turret Field Relative Angle", getFieldRelativeAngleDegrees());
            SmartDashboardTab.putNumber("Turret", "Turret Setpoint", getSetpoint());
    //    SmartDashboardTab.putNumber("Turret", "Turret Error", turretMotor.getClosedLoopError());
    //    SmartDashboardTab.putNumber("Turret", "Turret Controller Setpoint", turretMotor.getClosedLoopTarget());
    //    SmartDashboardTab.putString("Turret", "Turret Control Mode", turretMotor.getControlMode().toString());
    //    SmartDashboardTab.putNumber("Turret", "Turret IAccum", turretMotor.getIntegralAccumulator());
            SmartDashboardTab.putBoolean("Turret", "Home", getTurretHome());
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(getControlMode() == 1)
            setClosedLoopPosition();

        // TODO: FIX
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

    public double getTurretSimAngleDegrees(){
        return getTurretAngleDegrees() + 180;
    }

    public Pose2d getTurretSimPose() {
        return new Pose2d(m_driveTrain.getRobotPoseMeters().getX(),
                          m_driveTrain.getRobotPoseMeters().getY(),
                          new Rotation2d(Math.toRadians(getTurretSimAngleDegrees())));
    }


    @Override
    public void simulationPeriodic() {
    }

    public double getIdealTargetDistanceMeters() {
        return Math.sqrt(Math.pow(SimConstants.blueGoalPose.getY() - getTurretSimPose().getY(), 2) + Math.pow(SimConstants.blueGoalPose.getX() - getTurretSimPose().getX(), 2));
    }

    public double getIdealTurretAngleDegrees() {

        double targetRadians = Math.atan2(SimConstants.blueGoalPose.getY() - getTurretSimPose().getY(), SimConstants.blueGoalPose.getX() - getTurretSimPose().getX());

        return Math.toDegrees(targetRadians);
    }
}
