package frc.robot.modules;

import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class SystemStateHandler {
    public enum SystemState {
        TOP_CONE(ArmPosition.TOP_CONE, ArmPosition.AT_BUMPER, ElevatorPosition.TOP_CONE, true),
        TOP_CUBE(ArmPosition.TOP_CUBE, ArmPosition.AT_BUMPER, ElevatorPosition.TOP_CUBE, true),
        MID_CONE(ArmPosition.MID_CONE, ArmPosition.AT_BUMPER, ElevatorPosition.MID_CONE, true),
        MID_CUBE(ArmPosition.MID_CUBE, ArmPosition.AT_BUMPER, ElevatorPosition.MID_CUBE, true),
        FLOOR_CONE(ArmPosition.PAST_BUMPER, ArmPosition.PAST_BUMPER, ElevatorPosition.FLOOR_CONE, true),
        FLOOR_CUBE(ArmPosition.PAST_BUMPER, ArmPosition.PAST_BUMPER, ElevatorPosition.HOME, true),
        DOUBLE_SUBSTATION_CONE(ArmPosition.DOUBLE_SUBSTATION_CONE, ArmPosition.AT_BUMPER, ElevatorPosition.DOUBLE_SUBSTATION_CONE, true),
        DOUBLE_SUBSTATION_CUBE(ArmPosition.DOUBLE_SUBSTATION_CONE, ArmPosition.AT_BUMPER, ElevatorPosition.DOUBLE_SUBSTATION_CUBE, true),
        SINGLE_SUBSTATION_CONE(ArmPosition.AT_BUMPER, ArmPosition.AT_BUMPER, ElevatorPosition.SINGLE_SUBSTATION_CONE, false),
        SINGLE_SUBSTATION_CUBE(ArmPosition.AT_BUMPER, ArmPosition.AT_BUMPER, ElevatorPosition.SINGLE_SUBSTATION_CUBE, false),
        HOME(ArmPosition.HOME, ArmPosition.HOME, ElevatorPosition.HOME, false),
        CARRY(ArmPosition.AT_BUMPER, ArmPosition.AT_BUMPER, ElevatorPosition.HOME, false);

        public final ArmPosition armActionExtension;
        public final ArmPosition armBaseExtension;
        public final ElevatorPosition elevatorPosition;
        public final boolean rotateArmDown;

        SystemState(
            ArmPosition e_armActionExtension,
            ArmPosition e_armBaseExtension,
            ElevatorPosition e_elevatorPosition, 
            boolean e_rotateArmDown) {
            armActionExtension = e_armActionExtension;
            armBaseExtension = e_armBaseExtension;
            elevatorPosition = e_elevatorPosition;
            rotateArmDown = e_rotateArmDown;
        }
    }

    private SystemState superstructureState;
    private static SystemStateHandler superstructure;

    private SystemStateHandler() {
        superstructureState = SystemState.HOME;
    }

    public static SystemStateHandler getInstance() {
        if(superstructure == null) {
            superstructure = new SystemStateHandler();
        }
        return superstructure;
    }

    public SystemState getSystemState() {
        return superstructureState;
    }

    public double getDriveThrottle() {
        switch (superstructureState) {
            case TOP_CONE:
            case DOUBLE_SUBSTATION_CONE:
            case DOUBLE_SUBSTATION_CUBE:
            case TOP_CUBE:
            case MID_CONE:
            case MID_CUBE:
            case FLOOR_CONE:
            case FLOOR_CUBE:
            case SINGLE_SUBSTATION_CONE:
            case SINGLE_SUBSTATION_CUBE:
                return 0.6;
            case HOME:
            case CARRY:
            default:
                return 0.85;
        }
    }

    public double getRotationThrottle() {
        switch (superstructureState) {
            case TOP_CONE:
            case DOUBLE_SUBSTATION_CONE:
            case DOUBLE_SUBSTATION_CUBE:
            case TOP_CUBE:
            case MID_CONE:
            case MID_CUBE:
                return 0.6;
            case FLOOR_CONE:
            case FLOOR_CUBE:
            case SINGLE_SUBSTATION_CONE:
            case SINGLE_SUBSTATION_CUBE:
                return 0.8;
            case HOME:
            case CARRY:
            default:
                return 1;
        }
    }

    public void setSuperstructureState(SystemState superstructureState) {
        this.superstructureState = superstructureState;
    }
}
