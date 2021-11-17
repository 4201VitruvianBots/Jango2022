/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

import java.util.function.DoubleSupplier;

/**
 * TODO: Add description
 */
public class SetTurretSetpointFieldAbsoluteWithVisionOld extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Turret m_turret;
    private final Vision m_vision;
    private final DoubleSupplier m_xInput;
    private final DoubleSupplier m_yInput;
    private final double deadZone = 0.1;
    private final Timer timer = new Timer();
    private double setpoint;
    private boolean timeout = false;
    private boolean movedJoystick = false;
    private boolean direction, directionTripped;

    /**
     * TODO: Add description
     *
     * @param turret The turret used by this command.
     * @param vision The vision used by this command.
     * @param xInput
     * @param yInput
     */
    public SetTurretSetpointFieldAbsoluteWithVisionOld(Turret turret, Vision vision, DoubleSupplier xInput, DoubleSupplier yInput) {
        m_turret = turret;
        m_vision = vision;
        m_xInput = xInput;
        m_yInput = yInput;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(turret);
        addRequirements(vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(m_turret.getControlMode() == TurretConstants.ControlMode.CLOSED_LOOP_SET) {
            if((Math.pow(m_xInput.getAsDouble(), 2) + Math.pow(m_yInput.getAsDouble(), 2)) >= Math.pow(deadZone, 2)) {

                if(! directionTripped) {
                    direction = m_yInput.getAsDouble() < 0;
                    directionTripped = true;
                }

                if(direction) {
                    if(m_xInput.getAsDouble() >= 0)
                        setpoint = - Math.toDegrees(Math.atan2(- m_xInput.getAsDouble(), m_yInput.getAsDouble()));
                    else
                        setpoint = 360 - Math.toDegrees(Math.atan2(- m_xInput.getAsDouble(), m_yInput.getAsDouble()));

                    if(setpoint > m_turret.getMaxAngle()) {
                        setpoint -= 360;
                        direction = false;
                    }
                } else {
                    if(m_xInput.getAsDouble() < 0)
                        setpoint = Math.toDegrees(Math.atan2(m_xInput.getAsDouble(), m_yInput.getAsDouble()));
                    else
                        setpoint = - 360 + Math.toDegrees(Math.atan2(m_xInput.getAsDouble(), m_yInput.getAsDouble()));

                    if(setpoint < m_turret.getMinAngle()) {
                        direction = true;
                        setpoint += 360;
                    }
                }
                movedJoystick = true;
            } else if(m_vision.hasTarget()) {
                setpoint = m_turret.getTurretAngle() + m_vision.getTargetX();

                if(setpoint > m_turret.getMaxAngle())
                    setpoint -= 360;
                else if(setpoint < m_turret.getMinAngle())
                    setpoint += 360;

                if(timeout) {
                    timer.stop();
                    timer.reset();
                    timeout = false;
                }
            } else { //if you can't see the target for x seconds, then disable the limelight
                timer.start();
                timeout = true;
                if(timer.get() > 1) { //change value if needed
                    timer.stop();
                    timer.reset();
                    timeout = false;
                }
            }

            if(movedJoystick) {
                movedJoystick = false;
            }

            m_turret.setRobotCentricSetpointDegrees(setpoint);
        } else {
            m_turret.setPercentOutput(m_xInput.getAsDouble() * 0.2); //manual mode TODO: re-tune
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
