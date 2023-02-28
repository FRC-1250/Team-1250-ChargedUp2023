package frc.robot.modules;

import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class SystemStateHandler {
    public enum SystemState {
        TOP_CONE(ArmPosition.TOP_CONE, ElevatorPosition.TOP_CONE, true),
        TOP_CUBE(ArmPosition.TOP_CUBE, ElevatorPosition.TOP_CUBE, true),
        MID_CONE(ArmPosition.MID_CONE, ElevatorPosition.MID_CONE, true),
        MID_CUBE(ArmPosition.MID_CUBE, ElevatorPosition.MID_CUBE, true),
        HYBRID(ArmPosition.HYBRID, ElevatorPosition.HYBRID, true),
        FLOOR(ArmPosition.FLOOR, ElevatorPosition.FLOOR, true),
        HOME(ArmPosition.HOME, ElevatorPosition.HOME, false),
        SUBSTATION(ArmPosition.SUBSTATION, ElevatorPosition.SUBSTATION, true);

        public final ArmPosition armExtendActionPosition;
        public final ElevatorPosition elevatorPosition;
        public final boolean rotateArmDown;

        SystemState(ArmPosition e_armExtendActionPosition, ElevatorPosition e_elevatorPosition, boolean e_rotateArmDown) {
            armExtendActionPosition = e_armExtendActionPosition;
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
            case SUBSTATION:
            case TOP_CUBE:
                return 0.30;
            case MID_CONE:
            case MID_CUBE:
                return 0.4;
            case HYBRID:
            case FLOOR:
                return 0.5;
            case HOME:
            default:
                return 0.75;
        }
    }

    public void setSuperstructureState(SystemState superstructureState) {
        this.superstructureState = superstructureState;
    }
}
