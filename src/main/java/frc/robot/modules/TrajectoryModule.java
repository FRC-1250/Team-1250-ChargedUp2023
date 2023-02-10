package frc.robot.modules;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.FollowTrajectory;

/**
 * @See https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage
 */
public class TrajectoryModule {
  PIDController xController = new PIDController(5.5, 0, 0);
  PIDController yController = new PIDController(5.5, 0, 0);
  PIDController rotController = new PIDController(4, 0, 0);
  private final PPHolonomicDriveController holonomicDriveController;
  private final Drivetrain drivetrain;
  private final Field2d field2d;
  private TrajectorySet trajectorySet = new TrajectorySet();

  public TrajectoryModule(Field2d field2d, Drivetrain drivetrain) {
    rotController.enableContinuousInput(-Math.PI, Math.PI);
    holonomicDriveController = new PPHolonomicDriveController(xController, yController, rotController);
    this.drivetrain = drivetrain;
    this.field2d = field2d;
  }

  public FollowTrajectory getStraightCommand() {
    return new FollowTrajectory(trajectorySet.straight, holonomicDriveController, field2d, true, drivetrain);
  }

  public FollowTrajectory getStraightAndRotateCCWCommand() {
    return new FollowTrajectory(trajectorySet.straightAndRotateCCW, holonomicDriveController, field2d, true,
        drivetrain);
  }

  public FollowTrajectory getSnakeCommand() {
    return new FollowTrajectory(trajectorySet.snake, holonomicDriveController, field2d, true, drivetrain);
  }

  public FollowTrajectory getFigureEightCommand() {
    return new FollowTrajectory(trajectorySet.figureEight, holonomicDriveController, field2d, true, drivetrain);
  }

  public FollowTrajectory getForwardsTwoCommand() {
    return new FollowTrajectory(trajectorySet.forwardTwoMeters, holonomicDriveController, field2d, true, drivetrain);
  }
  public FollowTrajectory getBackwardsTwoCommand() {
    return new FollowTrajectory(trajectorySet.backTwoMeters, holonomicDriveController, field2d, true, drivetrain);
  }
  public FollowTrajectory getCornerTurnCommand() {
    return new FollowTrajectory(trajectorySet.cornerTurn, holonomicDriveController, field2d, true, drivetrain);
  }
  public FollowTrajectory getCCWRotationCommand() {
    return new FollowTrajectory(trajectorySet.CCWRotate, holonomicDriveController, field2d, true, drivetrain);
  }
  public FollowTrajectory getTestingPathCommand() {
    return new FollowTrajectory(trajectorySet.testingPath, holonomicDriveController, field2d, true, drivetrain);
  }
  public FollowTrajectory getBottomConeCubeCommand() {
    return new FollowTrajectory(trajectorySet.bottomConeCube, holonomicDriveController, field2d, true, drivetrain);
  }


  private class TrajectorySet {
    public final PathPlannerTrajectory straight;
    public final PathPlannerTrajectory straightAndRotateCCW;
    public final PathPlannerTrajectory snake;
    public final PathPlannerTrajectory figureEight;
    public final PathPlannerTrajectory forwardTwoMeters;
    public final PathPlannerTrajectory backTwoMeters;
    public final PathPlannerTrajectory cornerTurn;
    public final PathPlannerTrajectory CCWRotate;
    public final PathPlannerTrajectory testingPath;
    public final PathPlannerTrajectory bottomConeCube;

    public TrajectorySet() {
      straight = getStraight();
      straightAndRotateCCW = getStraightAndRotateCCW();
      snake = getSnake();
      figureEight = getFigureEight();
      forwardTwoMeters = getForwardsTwo();
      backTwoMeters = getBackwardsTwo();
      cornerTurn = getCornerTurn();
      CCWRotate = getCCWRotation();
      testingPath = getTestingPath();
      bottomConeCube = getBottomConeCube();
    }

    private PathPlannerTrajectory getStraightAndRotateCCW() {
      return PathPlanner.generatePath(
          new PathConstraints(Constants.DrivetrainCalibration.MAX_AUTO_DRIVE_SPEED, Constants.DrivetrainCalibration.MAX_DRIVE_ACCELERATION),
          new PathPoint(new Translation2d(3, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
          new PathPoint(new Translation2d(5, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(45)),
          new PathPoint(new Translation2d(7, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90)));
    }

    private PathPlannerTrajectory getStraight() {
      return PathPlanner.generatePath(
          new PathConstraints(Constants.DrivetrainCalibration.MAX_AUTO_DRIVE_SPEED, Constants.DrivetrainCalibration.MAX_DRIVE_ACCELERATION),
          new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
          new PathPoint(new Translation2d(2, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));
    }

    private PathPlannerTrajectory getSnake() {
      return PathPlanner.generatePath(
          new PathConstraints(Constants.DrivetrainCalibration.MAX_AUTO_DRIVE_SPEED, Constants.DrivetrainCalibration.MAX_DRIVE_ACCELERATION),
          new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
          new PathPoint(new Translation2d(1, 1), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
          new PathPoint(new Translation2d(2, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
          new PathPoint(new Translation2d(3, -1), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));
    }

    private PathPlannerTrajectory getFigureEight() {
      return PathPlanner.generatePath(
          new PathConstraints(Constants.DrivetrainCalibration.MAX_AUTO_DRIVE_SPEED, Constants.DrivetrainCalibration.MAX_DRIVE_ACCELERATION),
          new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
          new PathPoint(new Translation2d(1, 1), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
          new PathPoint(new Translation2d(2, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
          new PathPoint(new Translation2d(0, -0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
          new PathPoint(new Translation2d(-1, 1), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
          new PathPoint(new Translation2d(-2, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
          new PathPoint(new Translation2d(-1, -1), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
          new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));
    }

    private PathPlannerTrajectory getForwardsTwo() {
      return PathPlanner.loadPath("Forward Two Meters",
          new PathConstraints(Constants.DrivetrainCalibration.MAX_AUTO_DRIVE_SPEED, Constants.DrivetrainCalibration.MAX_DRIVE_ACCELERATION));
    }
    private PathPlannerTrajectory getBackwardsTwo() {
      return PathPlanner.loadPath("Back Two Meters",
          new PathConstraints(Constants.DrivetrainCalibration.MAX_AUTO_DRIVE_SPEED, Constants.DrivetrainCalibration.MAX_DRIVE_ACCELERATION));
    }
    private PathPlannerTrajectory getCornerTurn() {
      return PathPlanner.loadPath("Corner Turn",
          new PathConstraints(Constants.DrivetrainCalibration.MAX_AUTO_DRIVE_SPEED, Constants.DrivetrainCalibration.MAX_DRIVE_ACCELERATION));
    }
    private PathPlannerTrajectory getCCWRotation() {
      return PathPlanner.loadPath("CCW Rotation & Transition",
          new PathConstraints(Constants.DrivetrainCalibration.MAX_AUTO_DRIVE_SPEED, Constants.DrivetrainCalibration.MAX_DRIVE_ACCELERATION));
    }
    private PathPlannerTrajectory getTestingPath() {
      return PathPlanner.loadPath("Testing Path",
          new PathConstraints(Constants.DrivetrainCalibration.MAX_AUTO_DRIVE_SPEED, Constants.DrivetrainCalibration.MAX_DRIVE_ACCELERATION));
    }
    private PathPlannerTrajectory getBottomConeCube() {
      return PathPlanner.loadPath("Bottom Cone, Cube",
          new PathConstraints(Constants.DrivetrainCalibration.MAX_AUTO_DRIVE_SPEED, Constants.DrivetrainCalibration.MAX_DRIVE_ACCELERATION));
    }
  }
}