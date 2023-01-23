// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.RevPneumaticModule;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveSwerve;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Elevator elevator = new Elevator();
  private final RevPneumaticModule revPneumaticModule = new RevPneumaticModule(Constants.MIN_COMPRESSOR_PRESSURE, Constants.MAX_COMPRESSOR_PRESSURE);
<<<<<<< Updated upstream
=======
  private final Limelight limelight = new Limelight();
  private final XboxController xboxController = new XboxController(0);
  private final JoystickButton rbumper = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);
  private final Drivetrain m_swerve = new Drivetrain();
>>>>>>> Stashed changes

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */



   public RobotContainer() {
    configureButtonBindings();
    m_swerve.setDefaultCommand(
            new DriveSwerve(
                    xboxController::getRightBumper,
                    0.75,
                    xboxController::getLeftY,
                    xboxController::getLeftX,
                    xboxController::getRightX,
                    true,
                    m_swerve));
}
/**
 * Use this method to define your button->command mappings. Buttons can be
 * created by
 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
 * subclasses ({@link
 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
 * passing it to a
 * {@link JoystickButton}.
 */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
