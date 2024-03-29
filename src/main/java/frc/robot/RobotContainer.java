// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.PneumaticHubCalibrations;
import frc.robot.commands.Arm.ResetArmPosition;
import frc.robot.commands.Elevator.ResetElevatorPosition;
import frc.robot.commands.Swerve.DriveSwereAutoBalance;
import frc.robot.commands.Swerve.DriveSwerve;
import frc.robot.commands.Swerve.DriveSwerveOffsetCenter;
import frc.robot.commands.Swerve.DriveSwerveTargetLock;
import frc.robot.commands.Swerve.DriveSwerveThrottled;
import frc.robot.commands.Swerve.ResetPoseAndHeading;
import frc.robot.commands.Swerve.SwerveBrake;
import frc.robot.modules.CommandFactory;
import frc.robot.modules.SystemStateHandler;
import frc.robot.modules.TrajectoryBuilder;
import frc.robot.modules.TrajectoryModule;
import frc.robot.modules.SystemStateHandler.SystemState;
import frc.robot.modules.TrajectoryBuilder.TrajectoryLocation;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  private final PneumaticHub pneumaticHub = new PneumaticHub(PneumaticHubCalibrations.PNEUMATIC_HUB_ID);
  private final Limelight limelight = new Limelight();
  private final Drivetrain drivetrain = new Drivetrain();
  private final Elevator elevator = new Elevator(pneumaticHub);
  private final EndEffector endEffector = new EndEffector();
  private final Arm arm = new Arm(pneumaticHub);
  private final TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder();
  private final TrajectoryModule trajectoryModule = new TrajectoryModule();
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final CommandFactory commandFactory = new CommandFactory(arm, elevator, drivetrain, endEffector, limelight,
      trajectoryModule);

  private final XboxController driverXboxController = new XboxController(0);
  Trigger startButton = new Trigger(driverXboxController::getStartButton);
  Trigger backButton = new Trigger(driverXboxController::getBackButton);
  Trigger yButton = new Trigger(driverXboxController::getYButton);
  Trigger aButton = new Trigger(driverXboxController::getAButton);
  Trigger xButton = new Trigger(driverXboxController::getXButton);
  Trigger bButton = new Trigger(driverXboxController::getBButton);
  Trigger leftBumper = new Trigger(driverXboxController::getLeftBumper);
  Trigger rightBumper = new Trigger(driverXboxController::getRightBumper);
  Trigger leftStickButton = new Trigger(driverXboxController::getLeftStickButton);
  Trigger rightStickButton = new Trigger(driverXboxController::getRightStickButton);

  Trigger leftTrigger = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return driverXboxController.getLeftTriggerAxis() > 0.5;
    }
  });

  Trigger rightTrigger = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return driverXboxController.getRightTriggerAxis() > 0.5;
    }
  });

  Trigger pov = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return driverXboxController.getPOV() != -1;
    }
  });

  private final PS4Controller operatorPS4Controller = new PS4Controller(1);
  Trigger touchpad = new Trigger(operatorPS4Controller::getTouchpad);
  Trigger optionsButton = new Trigger(operatorPS4Controller::getOptionsButton);
  Trigger shareButton = new Trigger(operatorPS4Controller::getShareButton);
  Trigger crossButton = new Trigger(operatorPS4Controller::getCrossButton);
  Trigger triangleButton = new Trigger(operatorPS4Controller::getTriangleButton);
  Trigger squareButton = new Trigger(operatorPS4Controller::getSquareButton);
  Trigger circleButton = new Trigger(operatorPS4Controller::getCircleButton);
  Trigger psButton = new Trigger(operatorPS4Controller::getPSButton);
  Trigger l1Button = new Trigger(operatorPS4Controller::getL1Button);
  Trigger l2Button = new Trigger(operatorPS4Controller::getL2Button);
  Trigger l3Button = new Trigger(operatorPS4Controller::getL3Button);
  Trigger r1Button = new Trigger(operatorPS4Controller::getR1Button);
  Trigger r2Button = new Trigger(operatorPS4Controller::getR2Button);
  Trigger r3Button = new Trigger(operatorPS4Controller::getR3Button);

  Trigger upDpad = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return operatorPS4Controller.getPOV() == 0;
    }
  });

  Trigger rightDpad = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return operatorPS4Controller.getPOV() == 90;
    }
  });

  Trigger downDpad = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return operatorPS4Controller.getPOV() == 180;
    }
  });

  Trigger leftDpad = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return operatorPS4Controller.getPOV() == 270;
    }
  });

  // Joysticks defined here (Defined Joystick)
  Trigger leftJoystickUp = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return operatorPS4Controller.getLeftY() < -0.5;
    }
  });

  Trigger leftJoystickDown = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return operatorPS4Controller.getLeftY() > 0.5;
    }
  });

  Trigger leftJoystickRight = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return operatorPS4Controller.getLeftX() > 0.5;
    }
  });

  Trigger leftJoystickLeft = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return operatorPS4Controller.getLeftX() < -0.5;
    }
  });

  Trigger rightJoystickUp = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return operatorPS4Controller.getRightY() < -0.5;
    }
  });

  Trigger rightJoystickDown = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return operatorPS4Controller.getRightY() > 0.5;
    }
  });

  Trigger rightJoystickRight = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return operatorPS4Controller.getRightX() > 0.5;
    }
  });

  Trigger rightJoystickLeft = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return operatorPS4Controller.getRightX() < -0.5;
    }
  });

  public RobotContainer() {
    configureAutoCommands();
    configureButtonBindings();
    SmartDashboard.putData(drivetrain);
    SmartDashboard.putData(arm);
    SmartDashboard.putData(elevator);
    SmartDashboard.putData(endEffector);
  }

  private void configureButtonBindings() {

    /*
     * SmartDashboard controls
     */
    SmartDashboard.putData(new ResetArmPosition(arm));
    SmartDashboard.putData(new ResetElevatorPosition(elevator));
    SmartDashboard.putData(new ResetPoseAndHeading(drivetrain));

    /*
     * Driver controls
     */
    drivetrain.setDefaultCommand(
        new DriveSwerveThrottled(
            SystemStateHandler.getInstance()::getDriveThrottle,
            SystemStateHandler.getInstance()::getRotationThrottle,
            driverXboxController::getLeftY,
            driverXboxController::getLeftX,
            driverXboxController::getRightX,
            true,
            drivetrain));

    rightBumper.whileTrue(
      new DriveSwerveThrottled(
       () -> .6,
        SystemStateHandler.getInstance()::getRotationThrottle,
        driverXboxController::getLeftY,
        driverXboxController::getLeftX,
        driverXboxController::getRightX,
        true,
        drivetrain));

    pov.whileTrue(
        new DriveSwerveOffsetCenter(
            driverXboxController::getPOV,
            SystemStateHandler.getInstance()::getDriveThrottle,
            SystemStateHandler.getInstance()::getRotationThrottle,
            driverXboxController::getLeftY,
            driverXboxController::getLeftX,
            driverXboxController::getRightX,
            true,
            drivetrain));

    xButton.whileTrue(
        new DriveSwerveTargetLock(
            SystemStateHandler.getInstance()::getDriveThrottle,
            SystemStateHandler.getInstance()::getRotationThrottle,
            driverXboxController::getLeftY,
            driverXboxController::getLeftX,
            driverXboxController::getRightX,
            true,
            drivetrain,
            limelight));

    yButton.whileTrue(new DriveSwereAutoBalance(drivetrain));
    leftTrigger.whileTrue(commandFactory.endEffectorReleaseConeGraspCubeCommand());
    leftBumper.whileTrue(commandFactory.endEffectorReleaseCubeGraspConeCommand());
    aButton.whileTrue(new DriveSwereAutoBalance(drivetrain));
    bButton.whileTrue(new SwerveBrake(drivetrain));

    /*
     * Operator controls
     * Up and out is positive, Down and in is negative
     * Priotize automation over manual control
     */
    leftJoystickUp.whileTrue(commandFactory.setElevatorPercentOutputCommand(.5, false));
    leftJoystickDown.whileTrue(commandFactory.setElevatorPercentOutputCommand(0, false));
    rightJoystickRight.whileTrue(commandFactory.setArmPercentOutputCommand(0.75, false));
    rightJoystickLeft.whileTrue(commandFactory.setArmPercentOutputCommand(-0.75, false));
    shareButton.onTrue(commandFactory.rotateArmUpCommand());
    optionsButton.onTrue(commandFactory.rotateArmDownCommand());

    triangleButton.and(l1Button).onTrue(commandFactory.changeSystemStateCommand(SystemState.SINGLE_SUBSTATION_CONE));
    triangleButton.and(l2Button).onTrue(commandFactory.changeSystemStateCommand(SystemState.DOUBLE_SUBSTATION_CONE));
    triangleButton.and(upDpad).onTrue(commandFactory.changeSystemStateCommand(SystemState.TOP_CONE));
    triangleButton.and(leftDpad).onTrue(commandFactory.changeSystemStateCommand(SystemState.MID_CONE));
    triangleButton.and(downDpad).onTrue(commandFactory.changeSystemStateCommand(SystemState.FLOOR_CONE));
    squareButton.and(l1Button).onTrue(commandFactory.changeSystemStateCommand(SystemState.SINGLE_SUBSTATION_CUBE));
    squareButton.and(l2Button).onTrue(commandFactory.changeSystemStateCommand(SystemState.DOUBLE_SUBSTATION_CUBE));
    squareButton.and(upDpad).onTrue(commandFactory.changeSystemStateCommand(SystemState.TOP_CUBE));
    squareButton.and(leftDpad).onTrue(commandFactory.changeSystemStateCommand(SystemState.MID_CUBE));
    squareButton.and(downDpad).onTrue(commandFactory.changeSystemStateCommand(SystemState.FLOOR_CUBE));
    crossButton.onTrue(commandFactory.changeSystemStateCommand(SystemState.CARRY));
    circleButton.onTrue(commandFactory.extendArmBySystemStateCommand());
    circleButton.onFalse(commandFactory.retractArmBySystemStateCommand());
  }

  private void configureAutoCommands() {
    /*
     * Do nothing as default is a human safety condition, this should always be the
     * default
     */
    autoChooser.setDefaultOption("Do nothing", new WaitCommand(15));

    /*
     * It is safe to modify the X component of each added translation under the
     * endwith method (Positive = Forward, negative = Backward)
     * It is NOT SAFE to modify the Y component of each added translation under the
     * endwith method
     * The field is not truly mirroed so the Y translation must be mirrored as well
     * in order for a safe motion to occur
     */
    autoChooser.addOption(
        "Top Cone and mobility",
        Commands.sequence(
            commandFactory.autoScore(SystemState.TOP_CONE),
            commandFactory.changeSystemStateCommand(SystemState.CARRY),
            commandFactory.autoFollowPath(
                trajectoryBuilder
                    .startWith(TrajectoryLocation.BLUE_GRID_1)
                    .endWith(new PathPoint(
                        TrajectoryLocation.BLUE_GRID_1.translation2d.plus(new Translation2d(4.5, 0)),
                        Rotation2d.fromDegrees(0),
                        Rotation2d.fromDegrees(180)))
                    .build())));

    autoChooser.addOption(
        "Top Cube and mobility",
        Commands.sequence(
            commandFactory.autoScore(SystemState.TOP_CUBE),
            commandFactory.changeSystemStateCommand(SystemState.CARRY),
            commandFactory.autoFollowPath(
                trajectoryBuilder
                    .startWith(TrajectoryLocation.BLUE_GRID_2)
                    .endWith(new PathPoint(
                        TrajectoryLocation.BLUE_GRID_2.translation2d.plus(new Translation2d(4.5, 0)),
                        Rotation2d.fromDegrees(0),
                        Rotation2d.fromDegrees(180)))
                    .build())));

    autoChooser.addOption(
        "Top Cone and balance",
        Commands.sequence(
            commandFactory.autoScore(SystemState.TOP_CONE),
            commandFactory.changeSystemStateCommand(SystemState.CARRY),
            commandFactory.autoFollowPath(
                trajectoryBuilder
                    .startWith(TrajectoryLocation.BLUE_GRID_4)
                    .endWith(
                        new PathPoint(
                            TrajectoryLocation.BLUE_GRID_4.translation2d.plus(new Translation2d(1.5, 0)),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(180)))
                    .build()),
            new DriveSwereAutoBalance(drivetrain),
            new SwerveBrake(drivetrain)));

    autoChooser.addOption(
        "Top Cone and wait",
        Commands.sequence(
            commandFactory.autoScore(SystemState.TOP_CONE),
            commandFactory.changeSystemStateCommand(SystemState.CARRY)));

    autoChooser.addOption(
        "Top Cube and wait",
        Commands.sequence(
            commandFactory.autoScore(SystemState.TOP_CUBE),
            commandFactory.changeSystemStateCommand(SystemState.CARRY)));

    autoChooser.addOption(
        "Balance",
        Commands.sequence(
            commandFactory.autoFollowPath(
                trajectoryBuilder
                    .startWith(TrajectoryLocation.BLUE_GRID_4)
                    .endWith(
                        new PathPoint(
                            TrajectoryLocation.BLUE_GRID_4.translation2d.plus(new Translation2d(1.5, 0)),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(180)))
                    .build()),
            new DriveSwereAutoBalance(drivetrain),
            new SwerveBrake(drivetrain)));

    SmartDashboard.putData("Auto Chooser", autoChooser);
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
