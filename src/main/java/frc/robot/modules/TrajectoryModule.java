package frc.robot.modules;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;

/**
 * @See https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage
 */
public class TrajectoryModule {
    private final PIDController xController = new PIDController(5.5, 0, 0);
    private final PIDController yController = new PIDController(5.5, 0, 0);
    private final PIDController rotController = new PIDController(4, 0, 0);
    private final PPHolonomicDriveController holonomicDriveController;

    public TrajectoryModule() {
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        holonomicDriveController = new PPHolonomicDriveController(xController, yController, rotController);
    }

    public PPHolonomicDriveController getHolonomicDriveController() {
        return holonomicDriveController;
    }
}