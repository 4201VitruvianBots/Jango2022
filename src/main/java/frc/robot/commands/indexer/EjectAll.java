/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/**
 * Ejects all powercells from the robot out the intake.
 */
public class EjectAll extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Indexer m_indexer;
    private final Intake m_intake;

    /**
     * Ejects all powercells from the robot out the intake.
     * 
     * @param indexer The indexer used by the command.
     * @param intake The intake used by the command.
     */
    public EjectAll(Indexer indexer, Intake intake) {
        m_indexer = indexer;
        m_intake = intake;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(indexer);
        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_indexer.setIndexerPercentOutput(-0.6);
        m_indexer.setKickerPercentOutput(-0.5);
        m_intake.setIntakePercentOutput(-0.5);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
        m_indexer.setKickerPercentOutput(0);
        m_indexer.setIndexerPercentOutput(0);
        m_intake.setIntakePercentOutput(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
