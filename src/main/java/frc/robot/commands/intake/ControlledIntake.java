/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Intake.IntakeStates;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/**
 * Intakes balls until the indexer is full.
 */
public class ControlledIntake extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Indexer m_indexer;
    private final Intake m_intake;

    private final Joystick m_controller;
    private IntakeStates intakeState = IntakeStates.INTAKE_EMPTY;

    /**
     * Intakes balls until the indexer is full.
     *
     * @param intake The intake used by this command.
     * @param indexer The indexer used by this command.
     * @param controller Rumbles the controller when there are 5 balls.
     */
    public ControlledIntake(Intake intake, Indexer indexer, Joystick controller) {
        m_intake = intake;
        m_indexer = indexer;
        m_controller = controller;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake);
        addRequirements(indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intake.setIntakingState(true);

        if(m_indexer.getIntakeSensor() && m_indexer.getIndexerBottomSensor() && m_indexer.getIndexerTopSensor())
            intakeState = IntakeStates.INTAKE_FIVE_BALLS;
        else if(m_indexer.getIndexerBottomSensor() && m_indexer.getIndexerTopSensor())
            intakeState = IntakeStates.INTAKE_FOUR_BALLS;
        else
            intakeState = IntakeStates.INTAKE_ONE_BALL;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        switch(intakeState) {
            case INTAKE_FIVE_BALLS:
                m_intake.setIntakePercentOutput(0);
                m_indexer.setKickerPercentOutput(0);
                m_indexer.setIndexerPercentOutput(0);
                m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.4);
                m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.4);
                break;
            case INTAKE_FOUR_BALLS:
                m_intake.setIntakePercentOutput(0.8);
                m_indexer.setKickerPercentOutput(0);
                if(m_indexer.getIntakeSensor())
                    intakeState = IntakeStates.INTAKE_FIVE_BALLS;
                break;
            case INTAKE_ONE_BALL:
            default:
                m_intake.setIntakePercentOutput(0.8);
                m_indexer.setKickerPercentOutput(- 0.4);
                if(m_indexer.getIndexerBottomSensor()) {
                    m_indexer.setIndexerPercentOutput(0.95);
                } else {
                    m_indexer.setIndexerPercentOutput(0);
                }

                if(m_indexer.getIndexerTopSensor() && m_indexer.getIndexerBottomSensor()) {
                    m_indexer.setRPM(0);
                    intakeState = IntakeStates.INTAKE_FOUR_BALLS;
                }
                break;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        m_intake.setIntakingState(false);
        m_intake.setIntakePercentOutput(0);
        m_indexer.setIndexerPercentOutput(0);
        m_indexer.setKickerPercentOutput(0);
        if(intakeState == IntakeStates.INTAKE_FIVE_BALLS)
            m_intake.setintakePiston(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
