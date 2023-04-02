// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Slider;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_drive = new DriveTrain();
  private final Elevator m_elevator = new Elevator();
  private final Slider m_slider = new Slider();
  private final IntakeArm m_intakeArm = new IntakeArm();
  private final IntakeWheels m_intakeWheels = new IntakeWheels();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController1 =
      new CommandXboxController(OperatorConstants.kDriverControllerPort1);
  private final CommandXboxController m_driverController2 =
      new CommandXboxController(OperatorConstants.kDriverControllerPort2);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // Put the robot touching the community line, and move the drive train by the RobotLength distance.
    m_autoChooser.setDefaultOption("MobilityPoints", m_drive.driveDistanceCommand(40, 0.3));
    m_autoChooser.addOption("ScoreAndEngage", Autos.ScoreAndEngage(m_drive, m_elevator, m_slider, m_intakeArm, m_intakeWheels));
    m_autoChooser.addOption("ScoreAndMobilityShort", Autos.ScoreAndMobilityShort(m_drive, m_elevator, m_slider, m_intakeArm, m_intakeWheels));
    m_autoChooser.addOption("ScoreAndMobilityLong", Autos.ScoreAndMobilityLong(m_drive, m_elevator, m_slider, m_intakeArm, m_intakeWheels));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_elevator::elevatorIsNotSafe)
        .onTrue(m_elevator.stopCmd());
    new Trigger(m_intakeArm::armIsNotsafe)
        .onTrue(m_intakeArm.stopCmd());

    //set up the drivetrain command that runs all the time
    m_drive.setDefaultCommand(new RunCommand(
      () -> 
        m_drive.driveArcade(
          MathUtil.applyDeadband(- m_driverController1.getLeftY(), Constants.OperatorConstants.kDriveDeadband),
          MathUtil.applyDeadband(m_driverController1.getRightX()*Constants.DriveTrainConstants.kTurningScale, Constants.OperatorConstants.kDriveDeadband))
      , m_drive)
    );

    // Elevator and Slider
    new Trigger(() -> m_driverController2.getLeftY() > 0.75)
      .whileTrue(m_elevator.raise())
      .onFalse(m_elevator.stopCmd());
    
    new Trigger(() -> m_driverController2.getLeftY() < -0.75)
      .whileTrue(m_elevator.lower())
      .onFalse(m_elevator.stopCmd());
    
    new Trigger(() -> m_driverController2.getLeftX() > 0.75)
      .whileTrue(m_slider.slideOut())
      .onFalse(m_slider.stopCmd());
    
    new Trigger(() -> m_driverController2.getLeftX() < -0.75)
      .whileTrue(m_slider.slideIn())
      .onFalse(m_slider.stopCmd());
    
    // Intake Arm and Intake Wheels
    new Trigger(() -> m_driverController2.getRightY() > 0.75)
      .whileTrue(m_intakeArm.turnUp())
      .onFalse(m_intakeArm.stopCmd());
    
    new Trigger(() -> m_driverController2.getRightY() < -0.75)
      .whileTrue(m_intakeArm.turnDown())
      .onFalse(m_intakeArm.stopCmd());
    
    new Trigger(() -> m_driverController2.getRightX() > 0.75)
      .whileTrue(m_intakeWheels.grabCone())
      .onFalse(m_intakeWheels.stopCmd());

    new Trigger(() -> m_driverController2.getRightX() < -0.75)
      .whileTrue(m_intakeWheels.releaseCone())
      .onFalse(m_intakeWheels.stopCmd());

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController1.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return m_drive.driveDistanceCommand(5, 0.3);
    return m_autoChooser.getSelected();
  }
}
