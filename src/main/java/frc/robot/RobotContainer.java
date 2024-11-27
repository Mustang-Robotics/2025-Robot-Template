package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.FieldCentricDrive;
import frc.robot.commands.RobotCentricDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
    private final GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final SendableChooser<Command> m_chooser;

    public RobotContainer() {
        //Subsystem Initialization Functions
        m_chooser = AutoBuilder.buildAutoChooser();
        configureButtonBindings();
        m_robotDrive.calibrateGyro();
        m_robotDrive.SetRobotPose(5, 5, 90);
        // Configure default commands
        m_robotDrive.setDefaultCommand(
            new FieldCentricDrive(m_robotDrive, m_driverController));
        SmartDashboard.putData("Auto Chooser", m_chooser);
    }

    private void configureButtonBindings() {
        //Buttons mapping, try to make the buttons automate as much as possible
        new JoystickButton(m_driverController, Button.kX.value).whileTrue(
            new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
        new JoystickButton(m_driverController, Button.kA.value).toggleOnTrue(
            new SequentialCommandGroup(
                m_robotDrive.zeroHeading(), 
                new RobotCentricDrive(m_robotDrive, m_driverController)));
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
    }

}
