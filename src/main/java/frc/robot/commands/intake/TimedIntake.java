/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/**
 * Extends the pistons and runs the intake for a set amount of time.
 */
public class TimedIntake extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake m_intake;
    private final Indexer m_indexer;
    private final double m_time;
    private double startTime;

    /**
     * Extends the pistons and runs the intake for a set amount of time.
     * @param intake The intake used by this command.
     * @param indexer The indexer used by thsi command.
     * @param time The amount of time to run the command for.
     */
    public TimedIntake(Intake intake, Indexer indexer, double time) {
        m_intake = intake;
        m_indexer = indexer;
        m_time = time;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake);
        addRequirements(indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        if(m_intake.getIntakePistonExtendStatus() != true)
            m_intake.setintakePiston(true);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_indexer.setIndexerPercentOutput(1);
        m_indexer.setKickerPercentOutput(- 0.25);
        m_intake.setIntakePercentOutput(0.9);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_indexer.setIndexerPercentOutput(0);
        m_indexer.setKickerPercentOutput(0);
        m_intake.setIntakePercentOutput(0);
        if(m_intake.getIntakePistonExtendStatus() != false)
            m_intake.setintakePiston(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > m_time;
    }
}
