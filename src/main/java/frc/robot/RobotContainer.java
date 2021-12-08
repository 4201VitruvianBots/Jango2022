// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants;
import frc.robot.commands.autonomous.routines.AllyTrenchPathStraightSim;
import frc.robot.commands.climber.EnableClimbMode;
import frc.robot.commands.climber.SetClimberOutput;
import frc.robot.commands.drivetrain.SetArcadeDrive;
import frc.robot.commands.indexer.EjectAll;
import frc.robot.commands.intake.ControlledIntake;
import frc.robot.commands.intake.ToggleIntakePistons;
import frc.robot.commands.shooter.RapidFireSetpoint;
import frc.robot.commands.shooter.SetRpmSetpoint;
import frc.robot.commands.turret.SetTurretSetpointFieldAbsolute;
import frc.robot.commands.turret.ToggleTurretUsingSensor;
import frc.robot.commands.turret.ZeroTurretEncoder;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.vitruvianlib.utils.XBoxTrigger;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveTrain m_driveTrain = new DriveTrain();
    private final Intake m_intake = new Intake();
    private final Indexer m_indexer = new Indexer();
    private final Turret m_turret = new Turret(m_driveTrain);
    private final Vision m_vision = new Vision(m_driveTrain, m_turret);
    private final Shooter m_shooter = new Shooter(m_vision);
    private final Climber m_climber = new Climber();

    private FieldSim m_fieldSim;

    static Joystick leftJoystick = new Joystick(Constants.USB.leftJoystick);
    static Joystick rightJoystick = new Joystick(Constants.USB.rightJoystick);
    static Joystick xBoxController = new Joystick(Constants.USB.xBoxController);

    public Button[] leftButtons = new Button[2];
    public Button[] rightButtons = new Button[2];
    public Button[] xBoxButtons = new Button[10];
    public Button[] xBoxPOVButtons = new Button[8];
    public Button xBoxLeftTrigger, xBoxRightTrigger;

    private static boolean init = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        initializeSubsystems();

        // Configure the button bindings
        configureButtonBindings();
    }

    public static boolean getInitializationState() {
        return init;
    }

    public static void setInitializationState(boolean state) {
        init = state;
    }

    public void initializeSubsystems() {
        m_fieldSim = new FieldSim(m_driveTrain, m_turret);
        m_driveTrain.setDefaultCommand(
            new SetArcadeDrive(m_driveTrain,
                () -> -leftJoystick.getRawAxis(1),
                () -> rightJoystick.getRawAxis(0)));

        m_turret.setDefaultCommand(new SetTurretSetpointFieldAbsolute(m_turret, m_vision, m_shooter, m_climber, xBoxController));
        m_climber.setDefaultCommand(new SetClimberOutput(m_climber, xBoxController));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        for (int i = 0; i < leftButtons.length; i++)
            leftButtons[i] = new JoystickButton(leftJoystick, (i + 1));
        for (int i = 0; i < rightButtons.length; i++)
            rightButtons[i] = new JoystickButton(rightJoystick, (i + 1));
        for (int i = 0; i < xBoxButtons.length; i++)
            xBoxButtons[i] = new JoystickButton(xBoxController, (i + 1));
        for (int i = 0; i < xBoxPOVButtons.length; i++)
            xBoxPOVButtons[i] = new POVButton(xBoxController, (i * 45));

        xBoxLeftTrigger = new XBoxTrigger(xBoxController, 2);
        xBoxRightTrigger = new XBoxTrigger(xBoxController, 3);

        xBoxButtons[0].whenPressed(new SetRpmSetpoint(m_shooter, 3000, false));

        xBoxButtons[4].whenPressed(new ToggleIntakePistons(m_intake));
        xBoxLeftTrigger.whileHeld(new ControlledIntake(m_intake, m_indexer, xBoxController)); // Deploy intake

        // xBoxButtons[0].whileHeld(new SetRpmSetpoint(m_shooter, 3600, true)); // [A] Short-range
        // xBoxButtons[1].whileHeld(new SetRpmSetpoint(m_shooter, 3800, true)); // [B] Med-range
        // xBoxButtons[3].whileHeld(new SetRpmSetpoint(m_shooter, 4000, true)); // [Y] Long-range
        xBoxPOVButtons[0].whileHeld(new EjectAll(m_indexer, m_intake));      //Top POV - Eject All

        xBoxRightTrigger.whileHeld(new RapidFireSetpoint(m_shooter, m_indexer, m_intake));        // flywheel on toggle

        xBoxButtons[6].whenPressed(new ToggleTurretUsingSensor(m_turret));                        // start - toggle control mode turret
        xBoxButtons[9].whenPressed(new EnableClimbMode(m_climber, m_turret));                     // R3 - toggle driver climb mode?

        xBoxPOVButtons[4].whenPressed(new ZeroTurretEncoder(m_turret));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new AllyTrenchPathStraightSim(m_driveTrain, m_fieldSim);
    }


    public void autonomousInit() {
        if (RobotBase.isReal()) {
            m_driveTrain.resetEncoderCounts();
            m_driveTrain.resetOdometry(m_driveTrain.getRobotPoseMeters(), m_fieldSim.getRobotPoseMeters().getRotation());
        } else {
            m_fieldSim.initSim();
            m_driveTrain.resetEncoderCounts();
            m_driveTrain.resetOdometry(m_fieldSim.getRobotPoseMeters(), m_fieldSim.getRobotPoseMeters().getRotation());
        }
    }

    public DriveTrain getRobotDrive() {
        return m_driveTrain;
    }

    public void simulationInit() {
        m_fieldSim.initSim();
        //m_driveTrain.setSimPose(new Pose2d(5,5, new Rotation2d()));
    }

    public void simulationPeriodic() {
        if (!RobotState.isTest())
            m_fieldSim.simulationPeriodic();
    }
}
