package frc.robot.modules;

import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class SystemStateHandler {
    public enum SystemState {
        TOP_CONE(ArmPosition.TOP_CONE, ElevatorPosition.TOP_CONE, true, false),
        TOP_CUBE(ArmPosition.TOP_CUBE, ElevatorPosition.TOP_CUBE, true, false),
        MID_CONE(ArmPosition.MID_CONE, ElevatorPosition.MID_CONE, true, false),
        MID_CUBE(ArmPosition.MID_CUBE, ElevatorPosition.MID_CUBE, true, false),
        FLOOR_CONE(ArmPosition.BUMPER, ElevatorPosition.FLOOR, true, true),
        FLOOR_CUBE(ArmPosition.BUMPER, ElevatorPosition.FLOOR, true, true),
        DOUBLE_SUBSTATION_CONE(ArmPosition.BUMPER, ElevatorPosition.DOUBLE_SUBSTATION_CONE, true, false),
        DOUBLE_SUBSTATION_CUBE(ArmPosition.BUMPER, ElevatorPosition.DOUBLE_SUBSTATION_CUBE, true, false),
        SINGLE_SUBSTATION_CONE(ArmPosition.HOME, ElevatorPosition.SINGLE_SUBSTATION_CONE, false, false),
        SINGLE_SUBSTATION_CUBE(ArmPosition.HOME, ElevatorPosition.SINGLE_SUBSTATION_CUBE, false, false),
        HOME(ArmPosition.HOME, ElevatorPosition.HOME, false, false),
        CARRY(ArmPosition.HOME, ElevatorPosition.CARRY, true, true);

        public final ArmPosition armExtendActionPosition;
        public final ElevatorPosition elevatorPosition;
        public final boolean rotateArmDown;
        public final boolean preExtendArmBeyondBumper;

        SystemState(
            ArmPosition e_armExtendActionPosition, 
            ElevatorPosition e_elevatorPosition, 
            boolean e_rotateArmDown,
            boolean e_preExtendArmBeyondBumper) {
            armExtendActionPosition = e_armExtendActionPosition;
            elevatorPosition = e_elevatorPosition;
            rotateArmDown = e_rotateArmDown;
            preExtendArmBeyondBumper = e_preExtendArmBeyondBumper;
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
                return 0.30;
            case MID_CONE:
            case MID_CUBE:
                return 0.4;
            case CARRY:
            case FLOOR_CONE:
            case FLOOR_CUBE:
            case SINGLE_SUBSTATION_CONE:
            case SINGLE_SUBSTATION_CUBE:
                return 0.5;
            case HOME:
            default:
                return 0.75;
        }
    }

    public double getRotationThrottle() {
        switch (superstructureState) {
            case TOP_CONE:
            case DOUBLE_SUBSTATION_CONE:
            case DOUBLE_SUBSTATION_CUBE:
            case TOP_CUBE:
                return 0.4;
            case MID_CONE:
            case MID_CUBE:
                return 0.6;
            case CARRY:
            case FLOOR_CONE:
            case FLOOR_CUBE:
            case SINGLE_SUBSTATION_CONE:
            case SINGLE_SUBSTATION_CUBE:
                return 0.75;
            case HOME:
            default:
                return 1;
        }
    }

    public void setSuperstructureState(SystemState superstructureState) {
        this.superstructureState = superstructureState;
    }
}
