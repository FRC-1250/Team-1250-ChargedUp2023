// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.PneumaticHubCalibrations;
import frc.robot.commands.Arm.ResetArmPosition;
import frc.robot.commands.Arm.RotateArmDown;
import frc.robot.commands.Arm.RotateArmUp;
import frc.robot.commands.Elevator.ResetElevatorPosition;
import frc.robot.commands.Swerve.DriveSwerve;
import frc.robot.commands.Swerve.ResetPoseAndHeading;
import frc.robot.modules.CommandFactory;
import frc.robot.modules.SystemStateHandler;
import frc.robot.modules.TrajectoryModule;
import frc.robot.modules.SystemStateHandler.SystemState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  private final PneumaticHub pneumaticHub = new PneumaticHub(PneumaticHubCalibrations.PNEUMATIC_HUB_ID);
  private final Limelight limelight = new Limelight();
  private final Drivetrain drivetrain = new Drivetrain();
  private final Elevator elevator = new Elevator(pneumaticHub);
  private final EndEffector endEffector = new EndEffector();
  private final Arm arm = new Arm(pneumaticHub);

  private final TrajectoryModule trajectoryModule = new TrajectoryModule();
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final CommandFactory commandFactory = new CommandFactory(arm, elevator, drivetrain, endEffector, limelight, trajectoryModule);

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

  Trigger upLeftJoystick = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return operatorPS4Controller.getLeftY() < -0.5;
    }
  });

  Trigger downLeftJoystick = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return operatorPS4Controller.getLeftY() > 0.5;
    }
  });

  Trigger rightLeftJoystick = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return operatorPS4Controller.getLeftX() > 0.5;
    }
  });

  Trigger leftLeftJoystick = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return operatorPS4Controller.getLeftX() < -0.5;
    }
  });

  Trigger upRightJoystick = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return operatorPS4Controller.getRightY() < -0.5;
    }
  });

  Trigger downRightJoystick = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return operatorPS4Controller.getRightY() > 0.5;
    }
  });

  Trigger rightRightJoystick = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return operatorPS4Controller.getRightX() > 0.5;
    }
  });

  Trigger leftRightJoystick = new Trigger(new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return operatorPS4Controller.getRightX() < -0.5;
    }
  });

  public RobotContainer() {
    configureAutoCommands();
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    SmartDashboard.putData(new ResetArmPosition(arm));
    SmartDashboard.putData(new ResetElevatorPosition(elevator));
    SmartDashboard.putData(new ResetPoseAndHeading(drivetrain));
 
    // Driver
    drivetrain.setDefaultCommand(
        new DriveSwerve(
            driverXboxController::getPOV,
            rightBumper::getAsBoolean, // Boost
            SystemStateHandler.getInstance()::getDriveThrottle,
            driverXboxController::getLeftY,
            driverXboxController::getLeftX,
            driverXboxController::getRightX,
            true,
            drivetrain));

    xButton.whileTrue(commandFactory.trackAprilTagCommand());
    leftTrigger.whileTrue(commandFactory.endEffectorReleaseConeGraspCubeCommand());
    leftBumper.whileTrue(commandFactory.endEffectorReleaseCubeGraspConeCommand());
    
    // Operator
    // Up and out is positive, Down and in is negative
    upDpad.whileTrue(commandFactory.setElevatorPercentOutputCommand(0.5, true));
    downDpad.whileTrue(commandFactory.setElevatorPercentOutputCommand(0.0, true));
    rightDpad.whileTrue(commandFactory.setArmPercentOutputCommand(0.25, false));
    leftDpad.whileTrue(commandFactory.setArmPercentOutputCommand(-0.5, false));
    shareButton.onTrue(new RotateArmDown(arm));
    optionsButton.onTrue(new RotateArmUp(arm));
    
    r1Button.onTrue(commandFactory.changeSystemStateCommand(SystemState.TOP_CUBE));
    r2Button.onTrue(commandFactory.changeSystemStateCommand(SystemState.MID_CUBE));
    l1Button.onTrue(commandFactory.changeSystemStateCommand(SystemState.TOP_CONE));
    l2Button.onTrue(commandFactory.changeSystemStateCommand(SystemState.MID_CONE));
    crossButton.onTrue(commandFactory.changeSystemStateCommand(SystemState.HOME));
    squareButton.onTrue(commandFactory.changeSystemStateCommand(SystemState.FLOOR));
    circleButton.onTrue(commandFactory.changeSystemStateCommand(SystemState.DOUBLE_SUBSTATION));
    triangleButton.onTrue(commandFactory.extendArmBySystemStateCommand())
        .and(() -> SystemStateHandler.getInstance().getSystemState() != SystemState.HOME);
    triangleButton.onFalse(commandFactory.preextendCommand())
        .and(() -> SystemStateHandler.getInstance().getSystemState() != SystemState.HOME);
  }

  private void configureAutoCommands() {
    autoChooser.setDefaultOption("DoNothing", new WaitCommand(15));

    autoChooser.addOption(
        "TopConeAndWait",
        commandFactory.autoScore(SystemState.TOP_CONE));

    autoChooser.addOption(
        "TopCubeAndWait",
        commandFactory.autoScore(SystemState.TOP_CUBE));    

    autoChooser.addOption(
        "BlueLongSideMobility",
        commandFactory.autoFollowPath(trajectoryModule.getTrajectorySet().blueLongSideMobility));

    autoChooser.addOption(
        "BlueShortSideMobility",
        commandFactory.autoFollowPath(trajectoryModule.getTrajectorySet().blueShortSideMobility));

    autoChooser.addOption(
        "RedLongSideMobility",
        commandFactory.autoFollowPath(trajectoryModule.getTrajectorySet().redLongSideMobility));

    autoChooser.addOption(
        "RedShortSideMobility",
        commandFactory.autoFollowPath(trajectoryModule.getTrajectorySet().redShortSideMobility));
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
