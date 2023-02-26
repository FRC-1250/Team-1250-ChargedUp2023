package frc.robot.modules;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DrivetrainCalibration;

public class TrajectoryBuilder {
    public enum TrajectoryLocation {
        BLUE_GRID_1(new Translation2d(1.75, 0.5)),
        BLUE_GRID_2(new Translation2d(1.75, 1.10)),
        BLUE_GRID_3(new Translation2d(1.75, 1.60)),
        BLUE_GRID_4(new Translation2d(1.75, 2.10)),
        BLUE_GRID_5(new Translation2d(1.75, 2.80)),
        BLUE_GRID_6(new Translation2d(1.75, 3.30)),
        BLUE_GRID_7(new Translation2d(1.75, 3.80)),
        BLUE_GRID_8(new Translation2d(1.75, 4.40)),
        BLUE_GRID_9(new Translation2d(1.75, 5)),
        BLUE_FLOOR_1(new Translation2d(7.08, 0.95)),
        BLUE_FLOOR_2(new Translation2d(7.08, 2.16)),
        BLUE_FLOOR_3(new Translation2d(7.08, 3.37)),
        BLUE_FLOOR_4(new Translation2d(7.08, 4.58)),
        RED_GRID_1(new Translation2d(14.75, 0.5)),
        RED_GRID_2(new Translation2d(14.75, 1.10)),
        RED_GRID_3(new Translation2d(14.75, 1.60)),
        RED_GRID_4(new Translation2d(14.75, 2.10)),
        RED_GRID_5(new Translation2d(14.75, 2.80)),
        RED_GRID_6(new Translation2d(14.75, 3.30)),
        RED_GRID_7(new Translation2d(14.75, 3.80)),
        RED_GRID_8(new Translation2d(14.75, 4.40)),
        RED_GRID_9(new Translation2d(14.75, 5)),
        RED_FLOOR_1(new Translation2d(9.48, 0.95)),
        RED_FLOOR_2(new Translation2d(9.48, 2.16)),
        RED_FLOOR_3(new Translation2d(9.48, 3.37)),
        RED_FLOOR_4(new Translation2d(9.48, 4.58));

        public final Translation2d translation2d;

        TrajectoryLocation(Translation2d e_translation2d) {
            translation2d = e_translation2d;
        }
    }

    private final double X_COORDINATE_MIDPOINT = Units.feetToMeters(27);
    private PathPoint startPathPoint = null;
    private final List<PathPoint> pathPoints = new ArrayList<>();
    private PathPoint endPathPoint = null;

    private final PathConstraints pathConstraints = new PathConstraints(
            DrivetrainCalibration.MAX_AUTO_DRIVE_SPEED,
            DrivetrainCalibration.MAX_DRIVE_ACCELERATION);

    private final List<PathPoint> driveStraightAndRotateCounterClockwise = new ArrayList<>();
    private final List<PathPoint> blueShortSideMobility = new ArrayList<>();
    private final List<PathPoint> blueLongSideMobility = new ArrayList<>();
    private final List<PathPoint> redShortSideMobility = new ArrayList<>();
    private final List<PathPoint> redLongSideMobility = new ArrayList<>();
    private final List<PathPoint> blueShortSidePickup = new ArrayList<>();
    private final List<PathPoint> blueLongSidePickup = new ArrayList<>();
    private final List<PathPoint> redShortSidePickup = new ArrayList<>();
    private final List<PathPoint> redLongSidePickup = new ArrayList<>();


    public TrajectoryBuilder() {
        buildShortSideMobility();
        buildLongSideMobility();
        buildShortSidePickup();
        buildLongSidePickup();
        buildDriveStraightAndRotateCounterClockwise();
    }

    public TrajectoryBuilder driveStraightAndRotateCounterClockwise() {
        pathPoints.addAll(driveStraightAndRotateCounterClockwise);
        return this;
    }

    public TrajectoryBuilder blueShortSidePickup() {
        pathPoints.addAll(blueShortSidePickup);
        return this;
    }

    public TrajectoryBuilder blueLongSidePickup() {
        pathPoints.addAll(blueLongSidePickup);
        return this;
    }

    public TrajectoryBuilder blueShortSideMobility() {
        pathPoints.addAll(blueShortSideMobility);
        return this;
    }

    public TrajectoryBuilder blueShortSideMobilityReverse() {
        pathPoints.addAll(reverseList(blueShortSideMobility));
        return this;
    }

    public TrajectoryBuilder blueLongSideMobility() {
        pathPoints.addAll(blueLongSideMobility);
        return this;
    }

    public TrajectoryBuilder blueLongSideMobilityReverse() {
        pathPoints.addAll(reverseList(blueLongSideMobility));
        return this;
    }

    public TrajectoryBuilder redShortSidePickup() {
        pathPoints.addAll(redShortSidePickup);
        return this;
    }

    public TrajectoryBuilder redLongSidePickup() {
        pathPoints.addAll(redLongSidePickup);
        return this;
    }

    public TrajectoryBuilder redShortSideMobility() {
        pathPoints.addAll(redShortSideMobility);
        return this;
    }

    public TrajectoryBuilder redShortSideMobilityReverse() {
        pathPoints.addAll(reverseList(redShortSideMobility));
        return this;
    }

    public TrajectoryBuilder redLongSideMobility() {
        pathPoints.addAll(redLongSideMobility);
        return this;
    }

    public TrajectoryBuilder redLongSideMobilityReverse() {
        pathPoints.addAll(reverseList(redLongSideMobility));
        return this;
    }

    public TrajectoryBuilder startWith(PathPoint point) {
        startPathPoint = point;
        return this;
    }

    public TrajectoryBuilder startWith(TrajectoryLocation point, Rotation2d holonomicHeading) {
        startPathPoint = new PathPoint(point.translation2d, new Rotation2d(), holonomicHeading);
        return this;
    }

    public TrajectoryBuilder endWith(PathPoint point) {
        endPathPoint = point;
        return this;
    }

    public TrajectoryBuilder endWith(TrajectoryLocation point, Rotation2d holonomicHeading) {
        endPathPoint = new PathPoint(point.translation2d, new Rotation2d(), holonomicHeading);
        return this;
    }

    public PathPlannerTrajectory build() {
        if (startPathPoint != null)
            pathPoints.add(0, startPathPoint);

        if (endPathPoint != null) {
            pathPoints.add(endPathPoint);
        }

        List<PathPoint> temp = new ArrayList<>();
        temp.addAll(pathPoints);
        reset();
        return PathPlanner.generatePath(pathConstraints, temp);
    }

    private void reset() {
        pathPoints.clear();
        startPathPoint = null;
        endPathPoint = null;
    }

    private void buildDriveStraightAndRotateCounterClockwise() {
        Translation2d p1 = new Translation2d(2, 0);
        Translation2d p2 = new Translation2d(4, 0);

        driveStraightAndRotateCounterClockwise.addAll(
                Arrays.asList(
                        new PathPoint(p1, new Rotation2d(), Rotation2d.fromDegrees(0)),
                        new PathPoint(p2, new Rotation2d(), Rotation2d.fromDegrees(90))));
    }

    private void buildShortSideMobility() {
        Translation2d p1 = new Translation2d(3, 4.6);
        Translation2d p2 = new Translation2d(6, 4.6);

        blueShortSideMobility.addAll(
                Arrays.asList(
                        new PathPoint(p1, new Rotation2d(), Rotation2d.fromDegrees(-135)),
                        new PathPoint(p2, new Rotation2d(), Rotation2d.fromDegrees(-45))));

        redShortSideMobility.addAll(
                Arrays.asList(
                        new PathPoint(mirrorOverX(p1), new Rotation2d(), Rotation2d.fromDegrees(-135)),
                        new PathPoint(mirrorOverX(p2), new Rotation2d(), Rotation2d.fromDegrees(-45))));
    }

    private void buildLongSideMobility() {
        Translation2d p1 = new Translation2d(3, 0.95);
        Translation2d p2 = new Translation2d(5.7, 0.95);

        blueLongSideMobility.addAll(
                Arrays.asList(
                        new PathPoint(p1, new Rotation2d(), Rotation2d.fromDegrees(135)),
                        new PathPoint(p2, new Rotation2d(), Rotation2d.fromDegrees(45))));

        redLongSideMobility.addAll(
                Arrays.asList(
                        new PathPoint(mirrorOverX(p1), new Rotation2d(), Rotation2d.fromDegrees(135)),
                        new PathPoint(mirrorOverX(p2), new Rotation2d(), Rotation2d.fromDegrees(45))));
    }

    private void buildLongSidePickup() {
        blueLongSidePickup.addAll(
            Arrays.asList(
                new PathPoint(TrajectoryLocation.BLUE_FLOOR_1.translation2d, new Rotation2d(), Rotation2d.fromDegrees(0))
            )
        );

        redLongSidePickup.addAll(
            Arrays.asList(
                new PathPoint(TrajectoryLocation.RED_FLOOR_1.translation2d, new Rotation2d(), Rotation2d.fromDegrees(0))
            )
        );
    }

    private void buildShortSidePickup() {
        blueShortSidePickup.addAll(
            Arrays.asList(
                new PathPoint(TrajectoryLocation.BLUE_FLOOR_4.translation2d, new Rotation2d(), Rotation2d.fromDegrees(0))
            )
        );

        redShortSidePickup.addAll(
            Arrays.asList(
                new PathPoint(TrajectoryLocation.RED_FLOOR_4.translation2d, new Rotation2d(), Rotation2d.fromDegrees(0))
            )
        );
    }

    private List<PathPoint> reverseList(List<PathPoint> points) {
        List<PathPoint> reversedList = new ArrayList<>();
        for (int i = points.size(); i < 0; i--) {
            reversedList.add(points.get(i));
        }
        return reversedList;
    }

    private Translation2d mirrorOverX(Translation2d point) {
        return new Translation2d(((X_COORDINATE_MIDPOINT - point.getX()) + X_COORDINATE_MIDPOINT), point.getY());
    }
}