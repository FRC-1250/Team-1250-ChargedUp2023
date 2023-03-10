package frc.robot.modules;

import java.util.ArrayList;
import java.util.List;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.RotateArmDown;
import frc.robot.commands.Arm.RotateArmUp;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Arm.SetArmPositionBySystemState;
import frc.robot.commands.Arm.SetArmPercentOutput;
import frc.robot.commands.Elevator.SetElevatorPercentOutput;
import frc.robot.commands.Elevator.SetElevatorPosition;
import frc.robot.commands.EndEffector.SetEndEffectorSpeed;
import frc.robot.commands.Swerve.FollowTrajectory;
import frc.robot.commands.Swerve.TrackTarget;
import frc.robot.modules.SystemStateHandler.SystemState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Limelight;

public class CommandFactory {

    private final Arm arm;
    private final Elevator elevator;
    private final Drivetrain drivetrain;
    private final EndEffector endEffector;
    private final Limelight limelight;
    private final TrajectoryModule trajectoryModule;

    public CommandFactory(Arm arm, Elevator elevator, Drivetrain drivetrain, EndEffector endEffector,
            Limelight limelight, TrajectoryModule trajectoryModule) {
        this.arm = arm;
        this.elevator = elevator;
        this.drivetrain = drivetrain;
        this.endEffector = endEffector;
        this.limelight = limelight;
        this.trajectoryModule = trajectoryModule;
    }

    public Command changeSystemStateCommand(SystemState systemState) {
        List<Command> commands = new ArrayList<>();

        if (systemState.rotateArmDown == false) {
            commands.add(rotateArmUpCommand());
            commands.add(homeArmCommand());
            commands.add(new SetElevatorPosition(elevator, systemState));
        }

        if (systemState.rotateArmDown == true) {
            commands.add(new SetElevatorPosition(elevator, systemState));
            if(systemState.preExtendArm) {
                commands.add(preextendCommand());
            }
            commands.add(rotateArmDownCommand());
        }
        return Commands.sequence(commands.toArray(Command[]::new));
    }

    public Command rotateArmDownCommand() {
        return new RotateArmDown(arm);
    }

    public Command rotateArmUpCommand() {
        return new RotateArmUp(arm);
    }

    public Command extendArmBySystemStateCommand() {
        return new SetArmPositionBySystemState(arm);
    }

    public Command homeArmCommand() {
        return new SetArmPosition(arm, Arm.ArmPosition.HOME);
    }

    public Command preextendCommand() {
        return new SetArmPosition(arm, Arm.ArmPosition.PRE_EXTEND);
    }

    public Command armExtendByDefaultCommand() {
        return new SetArmPosition(arm, Arm.ArmPosition.HYBRID);
    }

    public Command endEffectorReleaseConeGraspCubeCommand() {
        return new SetEndEffectorSpeed(endEffector, 0.75);
    }

    public Command endEffectorReleaseCubeGraspConeCommand() {
        return new SetEndEffectorSpeed(endEffector, -0.75);
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory pathPlannerTrajectory) {
        return new FollowTrajectory(
                pathPlannerTrajectory,
                trajectoryModule.getHolonomicDriveController(),
                true,
                drivetrain);
    }

    public Command trackAprilTagCommand() {
        return new TrackTarget(limelight, drivetrain);
    }

    public Command setElevatorPercentOutputCommand(double percentOut, boolean override) {
        return new SetElevatorPercentOutput(elevator, percentOut, override);
    }

    public Command setArmPercentOutputCommand(double percentOut, boolean override) {
        return new SetArmPercentOutput(arm, percentOut, override);
    }

    // Auto commands
    public Command autoScore(SystemState systemState) {
        List<Command> commands = new ArrayList<>();
        commands.add(changeSystemStateCommand(systemState));
        commands.add(new WaitCommand(0.5));
        commands.add(extendArmBySystemStateCommand());

        switch (systemState) {
            case TOP_CONE:
            case MID_CONE:
                commands.add(endEffectorReleaseConeGraspCubeCommand().withTimeout(0.5));
            default:
                commands.add(endEffectorReleaseCubeGraspConeCommand().withTimeout(0.5));

        }
        commands.add(preextendCommand().withTimeout(1));
        return Commands.sequence(commands.toArray(Command[]::new));
    }

    public Command autoFollowPath(PathPlannerTrajectory pathPlannerTrajectory) {
        return followTrajectoryCommand(pathPlannerTrajectory);
    }
}
