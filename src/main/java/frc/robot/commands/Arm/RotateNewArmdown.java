package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Newarm;

public class RotateNewArmdown extends InstantCommand {
    
    private final Newarm cmdNewarm;

    public RotateNewArmdown(Newarm newarm) {
        cmdNewarm = newarm;
    }
    @Override
    public void initialize() {
        cmdNewarm.rotateArmDown();
    }
}
