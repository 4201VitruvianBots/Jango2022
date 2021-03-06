/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * Feeds to shooter when it's ready to shoot
 */
public class RapidFireSetpoint extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Shooter m_shooter;
    private final Indexer m_indexer;
    private final Intake m_intake;

    private double startTime;

    /**
     * Feeds to shooter when it's ready to shoot
     *
     */
    public RapidFireSetpoint(Shooter shooter, Indexer indexer, Intake intake) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_shooter = shooter;
        m_indexer = indexer;
        m_intake = intake;
        addRequirements(shooter);
        addRequirements(indexer);
        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(Math.abs(m_shooter.getRpm(0) - m_shooter.getRpmSetpoint()) <= 100 || Timer.getFPGATimestamp() - startTime > 0.5) {
            m_indexer.setIndexerPercentOutput(1);
            m_indexer.setKickerPercentOutput(1);
            m_intake.setIntakePercentOutput(1);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.setIntakePercentOutput(0);
        m_indexer.setIndexerPercentOutput(0);
        m_indexer.setKickerPercentOutput(0);
        m_shooter.setPercentOutput(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (false);
    }
}
