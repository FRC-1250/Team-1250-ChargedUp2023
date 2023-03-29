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
import frc.robot.commands.Arm.WaitForArm;
import frc.robot.commands.Arm.ExtendArmBySystemState;
import frc.robot.commands.Arm.RetractArmBySystemState;
import frc.robot.commands.Arm.SetArmPercentOutput;
import frc.robot.commands.Elevator.SetElevatorPercentOutput;
import frc.robot.commands.Elevator.SetElevatorPosition;
import frc.robot.commands.Elevator.WaitForElevator;
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
        if (systemState.rotateArmDown) {
            return Commands.parallel(
                    new SetElevatorPosition(elevator, systemState),
                    Commands.sequence(
                            new WaitCommand(0.04),
                            new SetArmPosition(arm, systemState.armBaseExtension),
                            rotateArmDownCommand()));
        } else {
            return Commands.parallel(
                    Commands.sequence(
                            rotateArmUpCommand(),
                            new SetArmPosition(arm, systemState.armBaseExtension)),
                    Commands.sequence(
                            new WaitCommand(0.04),
                            new SetElevatorPosition(elevator, systemState)));
        }
    }

    public Command handleHomeCarrySingleSub(SystemState requestedSystemState) {
        switch (requestedSystemState) {
            case TOP_CONE:
            case TOP_CUBE:
            case MID_CONE:
            case MID_CUBE:
            case DOUBLE_SUBSTATION_CONE:
            case DOUBLE_SUBSTATION_CUBE:
                return elevatorMotionArmDownArmMotion(requestedSystemState);
            case FLOOR_CONE:
            case FLOOR_CUBE:
                return elevatorMotionArmMotionArmDown(requestedSystemState);
            case SINGLE_SUBSTATION_CONE:
            case SINGLE_SUBSTATION_CUBE:
            case CARRY:
            case HOME:
                return new SetElevatorPosition(elevator, requestedSystemState);
            default:
                return Commands.none();
        }
    }

    public Command handleFloor(SystemState requestedSystemState) {
        switch (requestedSystemState) {
            case TOP_CONE:
            case TOP_CUBE:
            case MID_CONE:
            case MID_CUBE:
            case DOUBLE_SUBSTATION_CONE:
            case DOUBLE_SUBSTATION_CUBE:
                return elevatorMotionArmMotion(requestedSystemState);
            case SINGLE_SUBSTATION_CONE:
            case SINGLE_SUBSTATION_CUBE:
            case CARRY:
            case HOME:
                return elevatorMotionArmUpArmMotion(requestedSystemState);
            case FLOOR_CONE:
            case FLOOR_CUBE:
                return new SetElevatorPosition(elevator, requestedSystemState);
            default:
                return Commands.none();
        }
    }

    public Command handleTopMidDoubleSub(SystemState requestedSystemState) {
        switch (requestedSystemState) {
            case TOP_CONE:
            case TOP_CUBE:
            case MID_CONE:
            case MID_CUBE:
            case DOUBLE_SUBSTATION_CONE:
            case DOUBLE_SUBSTATION_CUBE:
            case FLOOR_CONE:
            case FLOOR_CUBE:
                return elevatorMotionArmMotion(requestedSystemState);
            case SINGLE_SUBSTATION_CONE:
            case SINGLE_SUBSTATION_CUBE:
            case CARRY:
            case HOME:
                return elevatorMotionArmMotionArmUp(requestedSystemState);
            default:
                return Commands.none();
        }
    }

    public Command elevatorMotionArmMotion(SystemState requestedSystemState) {
        return Commands.parallel(
                new SetElevatorPosition(elevator, requestedSystemState),
                new SetArmPosition(arm, requestedSystemState.armActionExtension))
                .withName("elevatorMotionArmMotion");
    }

    public Command elevatorMotionArmUpArmMotion(SystemState requestedSystemState) {
        return Commands.parallel(
                new SetElevatorPosition(elevator, requestedSystemState),
                Commands.sequence(
                        new RotateArmUp(arm),
                        new SetArmPosition(arm, requestedSystemState.armBaseExtension)))
                .withName("elevatorMotionArmUpArmMotion");
    }

    public Command elevatorMotionArmMotionArmUp(SystemState requestedSystemState) {
        return Commands.parallel(
                Commands.sequence(
                        new WaitForArm(arm, requestedSystemState.armBaseExtension, 0.4),
                        new SetElevatorPosition(elevator, requestedSystemState)),
                Commands.sequence(
                        new SetArmPosition(arm, requestedSystemState.armBaseExtension),
                        //new WaitCommand(0.2),
                        new RotateArmUp(arm)))
                .withName("elevatorMotionArmMotionArmUp");
    }

    public Command elevatorMotionArmDownArmMotion(SystemState requestedSystemState) {
        return Commands.parallel(
                new SetElevatorPosition(elevator, requestedSystemState),
                Commands.sequence(
                        new WaitForElevator(elevator, requestedSystemState.elevatorPosition, 0.25),
                        new RotateArmDown(arm),
                        new WaitCommand(0.1),
                        new SetArmPosition(arm, requestedSystemState.armBaseExtension)))
                .withName("elevatorMotionArmDownArmMotion");
    }

    public Command elevatorMotionArmMotionArmDown(SystemState requestedSystemState) {
        return Commands.parallel(
                new SetElevatorPosition(elevator, requestedSystemState),
                Commands.sequence(
                        new SetArmPosition(arm, requestedSystemState.armBaseExtension),
                        new RotateArmDown(arm)))
                .withName("elevatorMotionArmMotionArmDown");
    }

    public Command rotateArmDownCommand() {
        return new RotateArmDown(arm);
    }

    public Command rotateArmUpCommand() {
        return new RotateArmUp(arm);
    }

    public Command extendArmBySystemStateCommand() {
        return new ExtendArmBySystemState(arm);
    }

    public Command retractArmBySystemStateCommand() {
        return new RetractArmBySystemState(arm);
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
                break;
            default:
                commands.add(endEffectorReleaseCubeGraspConeCommand().withTimeout(0.5));
                break;

        }
        commands.add(retractArmBySystemStateCommand());
        return Commands.sequence(commands.toArray(Command[]::new));
    }

    public Command autoFollowPath(PathPlannerTrajectory pathPlannerTrajectory) {
        return followTrajectoryCommand(pathPlannerTrajectory);
    }
}
