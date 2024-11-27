package frc.robot;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.FieldCentricDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

//import edu.wpi.first.networktables.GenericEntry;

public class RobotContainer {
    GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);
    //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    //XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    SendableChooser<Command> m_chooser;
    public Command centerPathCommand;
    public Command speakerPathCommand;
    public Command stageBottomPathCommand;
    public Command stageMiddleCommand;
    public Command stageTopPathCommand;
    public Command AmpPathCommand;
    PIDController followPID = new PIDController(.02, 0, 0);

    public RobotContainer() {// Configure the button bindings
        //m_robotDrive.AutonomousBuilder();
        //Subsystem Initialization Functions
        buildPathCommands();

        //This needs to be fixed yet
        //
        //
        PPHolonomicDriveController.overrideRotationFeedback(null);
        //
        //
        
        configureButtonBindings();
        m_robotDrive.calibrateGyro();
        // Configure default commands
        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new FieldCentricDrive(m_robotDrive, m_driverController));
            //new RunCommand(
            //    () -> m_ShooterSubsystem.runFeeder(m_driverController.getRightTriggerAxis()), m_ShooterSubsystem);
        // m_climbingSubsystem.setDefaultCommand(
        //     new RunCommand(
        //         () -> m_climbingSubsystem.SetClimbSpeed(-m_driverController.getRawAxis(2) + m_driverController.getRawAxis(3)), m_climbingSubsystem));    
            

        

        //Efficient Commands
        
        //SmartDashboard.putData("Auto Chooser", m_chooser);
        
    }
    
    //private GenericEntry setPoint = m_arm.tab.add("setPoint", 90).getEntry();
    private void configureButtonBindings() { //NOTE: All button commands
         
        // new JoystickButton(m_driverController, Button.kX.value).whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
        //new JoystickButton(m_driverController, Button.kA.value).onTrue(new RunCommand(() -> m_arm.setGoal(setPoint.getDouble(0)), m_arm));
        //new JoystickButton(m_driverController, Button.kStart.value).toggleOnTrue(new IntakeNote(m_arm, m_intakeSubsystem));//new StartEndCommand(() -> m_ShooterSubsystem.runFeeder(.5),() -> m_ShooterSubsystem.runFeeder(0), m_ShooterSubsystem));
        


        // new JoystickButton(m_driverController, Button.kStart.value).whileTrue(new ParallelCommandGroup(new PassingShot(m_robotDrive, m_driverController, followPID), new SetArm(m_arm, 50), new SetShooterSpeed(m_ShooterSubsystem, m_bottom, 3500, 0, 0)));
        
        

        //new JoystickButton(m_driverController, Button.kBack.value).onTrue(new InitializePrepareShoot(ArmAdjustmentActiveTF, this, m_ShooterSubsystem, m_bottom));
        // new JoystickButton(m_driverController, Button.kRightBumper.value).toggleOnTrue(new ParallelCommandGroup(new SetArm(m_arm, 115), AmpPathCommand).andThen(new AmpShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom)));
        // new JoystickButton(m_driverController, Button.kY.value).onTrue(new RunCommand(() -> m_arm.setGoal(115), m_arm));
        //new JoystickButton(m_driverController, Button.kB.value).toggleOnTrue(stageBottomPathCommand.andThen(new AShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom)));
        //new JoystickButton(m_driverController, Button.kY.value).toggleOnTrue(stageMiddleCommand.andThen(new AShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom)));
        // new JoystickButton(m_driverController, Button.kA.value).toggleOnTrue(stageTopPathCommand.andThen(new AShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom)));
        //new JoystickButton(m_driverController, Button.kRightBumper.value).toggleOnTrue(new AmpShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom));

    }

    
    

    //private GenericEntry kp = m_ShooterSubsystem.tab.add("kp", 0).getEntry();
    //private GenericEntry ki = m_ShooterSubsystem.tab.add("ki", 0).getEntry();
    //private GenericEntry kd = m_ShooterSubsystem.tab.add("kd", 0).getEntry();
    //private GenericEntry PID_kp = m_arm.tab.add("PID_kp", 0.5).getEntry();
    //private GenericEntry PID_ki = m_arm.tab.add("PID_ki", 0.5).getEntry();
    //private GenericEntry PID_kd = m_arm.tab.add("PID_kd", 0).getEntry();



    private void buildPathCommands(){
        
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
