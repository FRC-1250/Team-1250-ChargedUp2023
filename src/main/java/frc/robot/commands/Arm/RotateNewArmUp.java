package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Newarm;

public class RotateNewArmUp extends InstantCommand {
    
    private final Newarm cmdNewarm;

    public RotateNewArmUp(Newarm newarm) {
        cmdNewarm = newarm;
    }
    @Override 
    public void initialize() {
        cmdNewarm.rotateArmUp();
    }
}
