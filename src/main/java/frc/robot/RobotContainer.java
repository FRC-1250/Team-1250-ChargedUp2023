// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RevPneumaticModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveSwerve;
import frc.robot.commands.ResetPoseAndHeading;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Arm.SetArmSpeed;
import frc.robot.modules.TrajectoryModule;
import frc.robot.subsystems.Arm;
import frc.robot.commands.TrackTarget;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  private final Drivetrain drivetrain = new Drivetrain();
  private final Elevator elevator = new Elevator();
  private final RevPneumaticModule revPneumaticModule = new RevPneumaticModule(
      Constants.RevPneumaticModuleCalibration.MIN_COMPRESSOR_PRESSURE,
      Constants.RevPneumaticModuleCalibration.MAX_COMPRESSOR_PRESSURE);
  private final Limelight limelight = new Limelight();
  private final Arm arm = new Arm();

  private final XboxController xboxController = new XboxController(0);
  Trigger backButton = new Trigger(xboxController::getBackButton);
  Trigger YButton = new Trigger(xboxController::getYButton);
  Trigger AButton = new Trigger(xboxController::getAButton);
  Trigger LeftBumper = new Trigger(xboxController::getLeftBumper);
  Trigger RightBumper = new Trigger(xboxController::getRightBumper);
  Trigger bButton = new Trigger(xboxController::getBButton);
  
  private final Field2d field2d = new Field2d();
  private final TrajectoryModule trajectoryModule = new TrajectoryModule(field2d, drivetrain);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  Trigger xButton = new Trigger(xboxController::getXButton);
  
  public RobotContainer() {
    configureAutoCommands();
    configureDefaultCommands();
    configureButtonBindings();
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

  private void configureAutoCommands() {
    autoChooser.setDefaultOption("Straight Forward", trajectoryModule.getForwardsTwoCommand());
    autoChooser.addOption("Straight Back", trajectoryModule.getBackwardsTwoCommand());
    autoChooser.addOption("Forward 90 Right", trajectoryModule.getCornerTurnCommand());
    autoChooser.addOption("CCWRotation", trajectoryModule.getCCWRotationCommand());
    autoChooser.addOption("New Testing Paths", trajectoryModule.getTestingPathCommand());
    SmartDashboard.putData(autoChooser);
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(
        new DriveSwerve(
            xboxController::getRightBumper,
            0.75,
            xboxController::getLeftY,
            xboxController::getLeftX,
            xboxController::getRightX,
            true,
            drivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
