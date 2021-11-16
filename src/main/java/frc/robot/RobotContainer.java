// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
import frc.robot.Constants.USBConstants;
import frc.robot.commands.autonomous.routines.AllyTrenchPathStraightSim;
import frc.robot.commands.drivetrain.SetArcadeDrive;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveTrain;
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

    private FieldSim m_fieldSim;

    static Joystick leftJoystick = new Joystick(USBConstants.leftJoystick);
    static Joystick rightJoystick = new Joystick(USBConstants.rightJoystick);
    static Joystick xBoxController = new Joystick(USBConstants.xBoxController);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        initializeSubsystems();

        // Configure the button bindings
        configureButtonBindings();
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
