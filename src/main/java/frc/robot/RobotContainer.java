// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.FlapSubsystem;
import frc.robot.subsystems.MecanumDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  // The robot's subsystems and commands are defined here...
  private final MecanumDrivetrain mecanumDrive = new MecanumDrivetrain();
  private final FlapSubsystem flapSubsystem = new FlapSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick m_driverController =
      new CommandJoystick(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("raiseClimb", climbSubsystem.climbRiseCommand());
    NamedCommands.registerCommand("lowerClimb", climbSubsystem.climbFallCommand());
    NamedCommands.registerCommand("raiseFlap", flapSubsystem.flapRiseCommand());
    NamedCommands.registerCommand("lowerFlap", flapSubsystem.flapFallCommand());

    // Configure the trigger bindings
    configureBindings();
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    mecanumDrive.setDefaultCommand(
        mecanumDrive.driveCommand(
            m_driverController::getX,
            m_driverController::getY,
            m_driverController::getTwist,
            m_driverController.trigger()::getAsBoolean));

    m_driverController.button(5).onTrue(climbSubsystem.climbRiseCommand());
    m_driverController.button(3).onTrue(climbSubsystem.climbFallCommand());
    m_driverController.button(6).onTrue(flapSubsystem.flapRiseCommand());
    m_driverController.button(4).onTrue(flapSubsystem.flapFallCommand());
    m_driverController.button(2).onTrue(mecanumDrive.resetGyroCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    return autoChooser.getSelected();
  }
}
