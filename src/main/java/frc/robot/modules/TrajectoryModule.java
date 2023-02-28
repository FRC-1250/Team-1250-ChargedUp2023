package frc.robot.modules;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.modules.TrajectoryBuilder.TrajectoryLocation;

/**
 * @See https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage
 */
public class TrajectoryModule {
    public class TrajectorySet {
        public final PathPlannerTrajectory blueShortSideMobility;
        public final PathPlannerTrajectory blueLongSideMobility;
        public final PathPlannerTrajectory blueShortSideMobilityAndPickup;
        public final PathPlannerTrajectory blueLongSideMobilityAndPickup;

        public final PathPlannerTrajectory redShortSideMobility;
        public final PathPlannerTrajectory redLongSideMobility;
        public final PathPlannerTrajectory redShortSideMobilityAndPickup;
        public final PathPlannerTrajectory redLongSideMobilityAndPickup;

        private final TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder();

        public TrajectorySet() {
            blueLongSideMobility = trajectoryBuilder
                    .startWith(TrajectoryLocation.BLUE_GRID_1, Rotation2d.fromDegrees(180))
                    .blueLongSideMobility()
                    .build();
            blueLongSideMobilityAndPickup = trajectoryBuilder
                    .startWith(TrajectoryLocation.BLUE_GRID_1, Rotation2d.fromDegrees(180))
                    .blueShortSideMobility()
                    .blueShortSidePickup()
                    .blueShortSideMobilityReverse()
                    .endWith(TrajectoryLocation.BLUE_GRID_2, Rotation2d.fromDegrees(180))
                    .build();
            blueShortSideMobility = trajectoryBuilder
                    .startWith(TrajectoryLocation.BLUE_GRID_9, Rotation2d.fromDegrees(180))
                    .blueShortSideMobility()
                    .build();
            blueShortSideMobilityAndPickup = trajectoryBuilder
                    .startWith(TrajectoryLocation.BLUE_GRID_9, Rotation2d.fromDegrees(180))
                    .blueShortSideMobility()
                    .blueShortSidePickup()
                    .blueShortSideMobilityReverse()
                    .endWith(TrajectoryLocation.BLUE_GRID_8, Rotation2d.fromDegrees(180))
                    .build();

            redLongSideMobility = trajectoryBuilder
                    .startWith(TrajectoryLocation.RED_GRID_1, Rotation2d.fromDegrees(180))
                    .redLongSideMobility()
                    .build();
            redLongSideMobilityAndPickup = trajectoryBuilder
                    .startWith(TrajectoryLocation.RED_GRID_1, Rotation2d.fromDegrees(180))
                    .redShortSideMobility()
                    .redShortSidePickup()
                    .redShortSideMobilityReverse()
                    .endWith(TrajectoryLocation.RED_GRID_2, Rotation2d.fromDegrees(180))
                    .build();
            redShortSideMobility = trajectoryBuilder
                    .startWith(TrajectoryLocation.RED_GRID_9, Rotation2d.fromDegrees(180))
                    .redShortSideMobility()
                    .build();
            redShortSideMobilityAndPickup = trajectoryBuilder
                    .startWith(TrajectoryLocation.RED_GRID_9, Rotation2d.fromDegrees(180))
                    .redShortSideMobility()
                    .redShortSidePickup()
                    .redShortSideMobilityReverse()
                    .endWith(TrajectoryLocation.RED_GRID_8, Rotation2d.fromDegrees(180))
                    .build();

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