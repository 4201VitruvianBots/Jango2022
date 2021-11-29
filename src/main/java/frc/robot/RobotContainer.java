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
import frc.robot.commands.drivetrain.SetArcadeDrive;
import frc.robot.commands.shooter.SetRpmSetpoint;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
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
    private final Turret m_turret = new Turret(m_driveTrain);
    private final Vision m_vision = new Vision(m_driveTrain, m_turret);
    private final Shooter m_shooter = new Shooter(m_vision);

    private FieldSim m_fieldSim;

    static Joystick leftJoystick = new Joystick(Constants.USB.leftJoystick);
    static Joystick rightJoystick = new Joystick(Constants.USB.rightJoystick);
    static Joystick xBoxController = new Joystick(Constants.USB.xBoxController);

    public Button[] leftButtons = new Button[2];
    public Button[] rightButtons = new Button[2];
    public Button[] xBoxButtons = new Button[10];
    public Button[] xBoxPOVButtons = new Button[8];

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
        if (RobotBase.isReal()) {
            m_driveTrain.setDefaultCommand(
                    new SetArcadeDrive(m_driveTrain,
                            () -> -leftJoystick.getRawAxis(1),
                            () -> rightJoystick.getRawAxis(0)));
        }
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

        xBoxButtons[0].whenPressed(new SetRpmSetpoint(m_shooter, 3000, false));
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
