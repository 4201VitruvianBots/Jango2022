/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/**
 * Command to manipulate the turret angle based on several inputs.
 * <p>
 * If there is a target, follow the target.<p>
 * If the joystick is being pushed, calculate the heading relative to the field.<p>
 * Else, retain the current angle.
 */
public class SetTurretSetpointFieldAbsolute extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Turret m_turret;
    private final Vision m_vision;
    private final Shooter m_shooter;
    private final Climber m_climber;
    private final Joystick m_controller;
    private final double deadZone = 0.5;
    private double setpoint;
    private boolean turning;
    private boolean joystickMoved;

    /**
     * Command to manipulate the turret angle based on several inputs.
     * <p>
     * If there is a target, follow the target.<p>
     * If the joystick is being pushed, calculate the heading relative to the field.<p>
     * Else, retain the current angle.
     *
     * @param turret The turret used by this command.
     * @param vision The vision used by this command.
     * @param shooter The shooter used by this command.
     * @param climber Only turns the turret if the robot is not climbing.
     * @param controller Rumbles the controller when on target.
     */
    public SetTurretSetpointFieldAbsolute(Turret turret, Vision vision,
                                          Shooter shooter, Climber climber, Joystick controller) {
        m_turret = turret;
        m_vision = vision;
        m_shooter = shooter;
        m_climber = climber;
        m_controller = controller;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(! m_climber.getClimbState()) {
            if(m_turret.getUsingSensor()) {
                // TODO: Add fine adjustment mode when shooting?
                if((Math.pow(m_controller.getRawAxis(0), 2) + Math.pow(m_controller.getRawAxis(1), 2)) >= Math.pow(deadZone, 2)) {
                    m_vision.ledsOn();
                    m_vision.setLastValidTargetTime();
                    joystickMoved = true;

                    if(m_controller.getRawAxis(0) >= 0)
                        setpoint = - Math.toDegrees(Math.atan2(- m_controller.getRawAxis(0), m_controller.getRawAxis(1)));
                    else
                        setpoint = Math.toDegrees(Math.atan2(m_controller.getRawAxis(0), m_controller.getRawAxis(1)));

                    if(setpoint > m_turret.getMaxAngle())
                        setpoint = m_turret.getMaxAngle();

                    if(setpoint < m_turret.getMinAngle())
                        setpoint = m_turret.getMinAngle();
                    if(m_vision.hasTarget() && Math.abs(m_vision.getFilteredTargetX()) < 20) {
                        m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.4);
                        m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.4);
                    }
                } else if(m_vision.hasTarget() && ! joystickMoved) {
                    if(! turning) {
                        m_vision.ledsOn();
                        setpoint = m_turret.getTurretAngle() + m_vision.getTargetX();

                        if(setpoint > m_turret.getMaxAngle()) {
                            setpoint = m_turret.getMaxAngle();
                        } else if(setpoint < m_turret.getMinAngle()) {
                            setpoint = m_turret.getMinAngle();
                        }
                    } else {
                        m_vision.ledsOff();
                        if(m_turret.onTarget())
                            turning = false;
                    }
                } else if(! m_vision.hasTarget() && ! joystickMoved) {
                    setpoint = m_turret.getTurretAngle();
                } else {
                    joystickMoved = false;
                    m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                    m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
                }

                if(m_shooter.canShoot()) {
                    m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.4);
                    m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.4);
                } else {
                    m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                    m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
                }
                m_turret.setRobotCentricSetpointDegrees(setpoint);
            } else {
                m_turret.setPercentOutput(m_controller.getRawAxis(0) * 0.2); //manual mode
            }
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