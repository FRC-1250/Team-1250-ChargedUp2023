package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Newarm {
    
    private final WPI_TalonFX talon = new WPI_TalonFX(Constants.ArmCalibrations.TALON_CAN_ID);
    private final Solenoid airBrake;
    private final DoubleSolenoid angleToggle;
    private final PneumaticHub pneumaticHub;
    //private boolean reverseLocked = false;
    //private Timer reverseLockTimer = new Timer();

    public Newarm(PneumaticHub sPneumaticHub) {
    pneumaticHub = sPneumaticHub;
    angleToggle = pneumaticHub.makeDoubleSolenoid(Constants.ArmCalibrations.ANGLE_SOLENOID_FORWARD_PORT, Constants.ArmCalibrations.ANGLE_SOLENOID_REVERSE_PORT);
    airBrake = pneumaticHub.makeSolenoid(Constants.ArmCalibrations.BRAKE_SOLENOID_PORT);
    //talon.setNeutralMode(NeutralMode.Brake);
   // talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    //talon.setInverted(true);

        

    }
    
}
