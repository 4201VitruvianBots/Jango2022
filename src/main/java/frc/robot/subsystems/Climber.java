// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private final DoubleSolenoid climbPiston = new DoubleSolenoid(Constants.pcmOne, Constants.Climber.climbPistonForward, Constants.Climber.climbPistonReverse);
    private final TalonFX climbMotor = new TalonFX(Constants.climbMotorA);
    private final VictorSPX skyhookMotor = new VictorSPX(Constants.skyhookMotor);
    private boolean climbState;

    /**
     * Creates a new Climber.
     */
    public Climber() {
        // Set up climber motor
        climbMotor.configFactoryDefault();
        climbMotor.setSelectedSensorPosition(0);
        climbMotor.setNeutralMode(NeutralMode.Brake);
        climbMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        skyhookMotor.configFactoryDefault();
        skyhookMotor.setNeutralMode(NeutralMode.Brake);
        skyhookMotor.setInverted(true);
    }

    public boolean getClimbPistonExtendStatus() {
        return climbPiston.get() == DoubleSolenoid.Value.kForward;
    }

    /**
     * sets the state of the climb piston
     *
     * @param state true sets climber to go up. false sets climber to go down
     */
    public void setClimbPiston(boolean state) {
        climbPiston.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    /**
     * returns the state of the climb piston
     *
     * @return up is true. down is false
     */
    public boolean getClimbState() {
        return climbState;
    }

    /**
     * returns the state of the climb piston
     *
     * @param state up is true. down is false
     */
    public void setClimbState(boolean state) {
        this.climbState = state;
    }

    /**
     * prevents backdrive
     * @param value
     */
    public void setClimberOutput(double value) {
        climbMotor.set(ControlMode.PercentOutput, value);
    }

    public int getClimberPosition() {
        return climbMotor.getSelectedSensorPosition();
    }

    public void setSkyhook(double value) {
        skyhookMotor.set(ControlMode.PercentOutput, value);
    }

    private void updateShuffleboard() {
        SmartDashboard.putBoolean("Climb Mode", getClimbState());
        SmartDashboardTab.putBoolean("Climber", "Climb Mode", climbState);
        SmartDashboardTab.putBoolean("Climber", "Climb Pistons", getClimbPistonExtendStatus());

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
