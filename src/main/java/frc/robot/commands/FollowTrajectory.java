// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectory extends CommandBase {
  private final Drivetrain drivetrain;
  private final Timer timer = new Timer();
  private final PathPlannerTrajectory trajectory;
  private final PPHolonomicDriveController controller;
  private PathPlannerState desiredState;
  private ChassisSpeeds desiredChasisSpeeds;
  private final Field2d field2d;
  private final boolean resetPose;

  public FollowTrajectory(PathPlannerTrajectory trajectory, PPHolonomicDriveController controller, Field2d field2d, boolean resetPose, Drivetrain drivetrain) {
    this.trajectory = trajectory;
    this.controller = controller;
    this.field2d = field2d;
    this.drivetrain = drivetrain;
    this.resetPose = resetPose;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    field2d.getRobotObject().setTrajectory(trajectory);
    timer.reset();
    timer.start();

    if(resetPose) {
      drivetrain.resetOdometry(trajectory.getInitialHolonomicPose());
    }
  }

  @Override
  public void execute() {
    var pose = drivetrain.getPose();
    field2d.getRobotObject().setPose(pose);
    SmartDashboard.putData(field2d);

    desiredState = (PathPlannerState) trajectory.sample(timer.get());
    desiredChasisSpeeds = this.controller.calculate(pose, desiredState);
    drivetrain.setModuleStates(Constants.DrivetrainCalibration.KINEMATICS.toSwerveModuleStates(desiredChasisSpeeds));
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}
