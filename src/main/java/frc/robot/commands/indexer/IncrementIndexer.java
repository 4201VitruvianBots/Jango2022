/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

/**
 * Shifts all powercells up by running the indexer.
 */
public class IncrementIndexer extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Indexer m_indexer;
    private double startTime;
    
    /**
     * Shifts all powercells up by running the indexer.
     * @param indexer The indexer used by this command.
     */
    public IncrementIndexer(Indexer indexer) {
        m_indexer = indexer;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(indexer);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_indexer.setKickerPercentOutput(-0.2);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
        m_indexer.setKickerPercentOutput(0);
        SmartDashboard.putNumber("Execution Time", Timer.getFPGATimestamp() - startTime);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
