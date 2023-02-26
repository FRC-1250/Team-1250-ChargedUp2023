package frc.robot.modules;

import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class SystemStateHandler {
    public enum SystemState {
        TOP_CONE(ArmPosition.TOP_NODE, ElevatorPosition.TOP_CONE, true),
        TOP_CUBE(ArmPosition.TOP_NODE, ElevatorPosition.TOP_CUBE, true),
        MID_CONE(ArmPosition.MID_NODE, ElevatorPosition.MID_CONE, true),
        MID_CUBE(ArmPosition.MID_NODE, ElevatorPosition.MID_CUBE, true),
        HYBIRD(ArmPosition.HYBRID, ElevatorPosition.HYBIRD, true),
        FLOOR(ArmPosition.FLOOR, ElevatorPosition.FLOOR, true),
        HOME(ArmPosition.HOME, ElevatorPosition.HOME, false),
        SUBSTATION(ArmPosition.SUBSTATION, ElevatorPosition.SUBSTATION, true);

        public final ArmPosition armExtendActionPosition;
        public final ElevatorPosition elevatorPosition;
        public final boolean rotateArmForwardInMotion;

        SystemState(ArmPosition e_armExtendActionPosition, ElevatorPosition e_elevatorPosition, boolean e_rotateArmForwardInMotion) {
            armExtendActionPosition = e_armExtendActionPosition;
            elevatorPosition = e_elevatorPosition;
            rotateArmForwardInMotion = e_rotateArmForwardInMotion;
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
            case HOME:
                return 0.75;
            default:
                return 0.375;
        }
    }

    public void setSuperstructureState(SystemState superstructureState) {
        this.superstructureState = superstructureState;
    }
}
