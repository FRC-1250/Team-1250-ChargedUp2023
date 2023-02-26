package frc.robot.modules;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.modules.TrajectoryBuilder.TrajectoryLocation;

/**
 * @See https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage
 */
public class TrajectoryModule {
    public class TrajectorySet {
        public final PathPlannerTrajectory straightAndRotateCCW;
        public final PathPlannerTrajectory shortSideMobility;
        public final PathPlannerTrajectory longSideMobility;
        public final PathPlannerTrajectory shortSideMobilityAndPickup;
        public final PathPlannerTrajectory longSideMobilityAndPickup;
        private final TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder();

        public TrajectorySet() {
            switch (DriverStation.getAlliance()) {
                case Blue:
                    longSideMobility = trajectoryBuilder
                            .startWith(TrajectoryLocation.BLUE_GRID_1, Rotation2d.fromDegrees(180))
                            .blueLongSideMobility()
                            .build();
                    longSideMobilityAndPickup = trajectoryBuilder
                            .startWith(TrajectoryLocation.BLUE_GRID_1, Rotation2d.fromDegrees(180))
                            .blueShortSideMobility()
                            .blueShortSidePickup()
                            .blueShortSideMobilityReverse()
                            .endWith(TrajectoryLocation.BLUE_GRID_2, Rotation2d.fromDegrees(180))
                            .build();
                    shortSideMobility = trajectoryBuilder
                            .startWith(TrajectoryLocation.BLUE_GRID_9, Rotation2d.fromDegrees(180))
                            .blueShortSideMobility()
                            .build();
                    shortSideMobilityAndPickup = trajectoryBuilder
                            .startWith(TrajectoryLocation.BLUE_GRID_9, Rotation2d.fromDegrees(180))
                            .blueShortSideMobility()
                            .blueShortSidePickup()
                            .blueShortSideMobilityReverse()
                            .endWith(TrajectoryLocation.BLUE_GRID_8, Rotation2d.fromDegrees(180))
                            .build();
                    break;
                case Red:
                    longSideMobility = trajectoryBuilder
                            .startWith(TrajectoryLocation.RED_GRID_1, Rotation2d.fromDegrees(180))
                            .redLongSideMobility()
                            .build();
                    longSideMobilityAndPickup = trajectoryBuilder
                            .startWith(TrajectoryLocation.RED_GRID_1, Rotation2d.fromDegrees(180))
                            .redShortSideMobility()
                            .redShortSidePickup()
                            .redShortSideMobilityReverse()
                            .endWith(TrajectoryLocation.RED_GRID_2, Rotation2d.fromDegrees(180))
                            .build();
                    shortSideMobility = trajectoryBuilder
                            .startWith(TrajectoryLocation.RED_GRID_9, Rotation2d.fromDegrees(180))
                            .redShortSideMobility()
                            .build();
                    shortSideMobilityAndPickup = trajectoryBuilder
                            .startWith(TrajectoryLocation.RED_GRID_9, Rotation2d.fromDegrees(180))
                            .redShortSideMobility()
                            .redShortSidePickup()
                            .redShortSideMobilityReverse()
                            .endWith(TrajectoryLocation.RED_GRID_8, Rotation2d.fromDegrees(180))
                            .build();
                    break;
                default:
                    longSideMobility = new PathPlannerTrajectory();
                    longSideMobilityAndPickup = new PathPlannerTrajectory();
                    shortSideMobility = new PathPlannerTrajectory();
                    shortSideMobilityAndPickup = new PathPlannerTrajectory();
                    break;
            }
            straightAndRotateCCW = trajectoryBuilder.driveStraightAndRotateCounterClockwise().build();
        }
    }

    private final PIDController xController = new PIDController(5.5, 0, 0);
    private final PIDController yController = new PIDController(5.5, 0, 0);
    private final PIDController rotController = new PIDController(4, 0, 0);
    private final PPHolonomicDriveController holonomicDriveController;
    private final TrajectorySet trajectorySet;

    public TrajectoryModule() {
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        holonomicDriveController = new PPHolonomicDriveController(xController, yController, rotController);
        trajectorySet = new TrajectorySet();
    }

    public PPHolonomicDriveController getHolonomicDriveController() {
        return holonomicDriveController;
    }

    public TrajectorySet getTrajectorySet() {
        return trajectorySet;
    }
}