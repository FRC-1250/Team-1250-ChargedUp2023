package frc.robot.modules;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class SwerveModuleTalonFX {
    private final WPI_TalonFX driveTalon;
    private final WPI_TalonFX turningTalon;
    private final CANCoder canCoder;

    public SwerveModuleTalonFX(int driveTalonCanID, int turningTalonCanID, int canCoderCanID, double canCoderOffsetDegrees) {
        driveTalon = new WPI_TalonFX(driveTalonCanID, Constants.CANIVORE_BUS_NAME);
        canCoder = new CANCoder(canCoderCanID, Constants.CANIVORE_BUS_NAME);
        turningTalon = new WPI_TalonFX(turningTalonCanID, Constants.CANIVORE_BUS_NAME);

        configureDriveTalon();
        configureCanCoder(canCoderOffsetDegrees);
        configureTurningTalon();
    }

    private void configureDriveTalon() {
        driveTalon.configFactoryDefault();
        driveTalon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.CONFIG_TIMEOUT_MS);
        driveTalon.configClosedloopRamp(0.5, Constants.CONFIG_TIMEOUT_MS);
        driveTalon.config_kF(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.DrivetrainCalibration.DRIVE_TALON_VELOCITY_GAINS.kF, Constants.CONFIG_TIMEOUT_MS);
        driveTalon.config_kP(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.DrivetrainCalibration.DRIVE_TALON_VELOCITY_GAINS.kP, Constants.CONFIG_TIMEOUT_MS);
        driveTalon.config_kI(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.DrivetrainCalibration.DRIVE_TALON_VELOCITY_GAINS.kI, Constants.CONFIG_TIMEOUT_MS);
        driveTalon.config_kD(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.DrivetrainCalibration.DRIVE_TALON_VELOCITY_GAINS.kD, Constants.CONFIG_TIMEOUT_MS);
        driveTalon.setNeutralMode(NeutralMode.Brake);
        driveTalon.setSelectedSensorPosition(0, Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.CONFIG_TIMEOUT_MS);
        driveTalon.configAllowableClosedloopError(Constants.TALONFX_PRIMARY_PID_LOOP_ID, 100, Constants.CONFIG_TIMEOUT_MS);
    }

    private void configureCanCoder(double canCoderOffsetDegrees) {
        canCoder.configFactoryDefault();
        canCoder.configMagnetOffset(canCoderOffsetDegrees, Constants.CONFIG_TIMEOUT_MS);
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180, Constants.CONFIG_TIMEOUT_MS);
        canCoder.configSensorDirection(false, Constants.CONFIG_TIMEOUT_MS);
        canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, Constants.CONFIG_TIMEOUT_MS);
    }

    private void configureTurningTalon() {
        turningTalon.configFactoryDefault();
        turningTalon.setSensorPhase(true);
        turningTalon.configRemoteFeedbackFilter(canCoder, 0, Constants.CONFIG_TIMEOUT_MS);
        turningTalon.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.CONFIG_TIMEOUT_MS);
        turningTalon.configClosedloopRamp(0, Constants.CONFIG_TIMEOUT_MS);
        turningTalon.setInverted(TalonFXInvertType.CounterClockwise);
        turningTalon.config_kF(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.DrivetrainCalibration.TURNING_TALON_POSITION_GAINS.kF, Constants.CONFIG_TIMEOUT_MS);
        turningTalon.config_kP(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.DrivetrainCalibration.TURNING_TALON_POSITION_GAINS.kP, Constants.CONFIG_TIMEOUT_MS);
        turningTalon.config_kI(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.DrivetrainCalibration.TURNING_TALON_POSITION_GAINS.kI, Constants.CONFIG_TIMEOUT_MS);
        turningTalon.config_kD(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.DrivetrainCalibration.TURNING_TALON_POSITION_GAINS.kD, Constants.CONFIG_TIMEOUT_MS);
        turningTalon.configAllowableClosedloopError(Constants.TALONFX_PRIMARY_PID_LOOP_ID, 100, Constants.CONFIG_TIMEOUT_MS);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveTalon.getSelectedSensorVelocity(), getFromHeading());
    }

    public SwerveModulePosition getPosition() {
        double wheelRadius = Units.inchesToMeters(2);
        double talonFXintegratedSensorResolution = 2048; 
        double mk4iL1DriveRatio = 1 / 8.14; // Drive gear ratio for SES mk4i module
        double RotationToDistanceMath = 2*Math.PI*wheelRadius/talonFXintegratedSensorResolution*mk4iL1DriveRatio;
        return new SwerveModulePosition(driveTalon.getSelectedSensorPosition() * RotationToDistanceMath, getFromHeading());
    }
    
    public void resetPosition() {
        driveTalon.setSelectedSensorPosition(0, Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.CONFIG_TIMEOUT_MS);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d canCoderHeading = getFromHeading();
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, canCoderHeading);
        var talonPos = turningTalon.getSelectedSensorPosition();
        var turnChange = -determineActualTurnChange(state.angle, canCoderHeading) * Constants.DrivetrainCalibration.DEGRESS_TO_TALON_TICKS_CONVERSION_FACTOR;
        driveTalon.set(TalonFXControlMode.Velocity, state.speedMetersPerSecond * Constants.DrivetrainCalibration.METERS_PER_SECOND_TO_TALON_TICKS_CONVERSION_FACTOR);
        turningTalon.set(TalonFXControlMode.Position, turnChange + talonPos);
    }

    private double determineActualTurnChange(Rotation2d optimizedAngle, Rotation2d currentAngle) {
        var actualDegreeChange = optimizedAngle.minus(currentAngle).getDegrees();
    
        // Rotation 2d minus can result in some very small numbers, throw those out.
        if (actualDegreeChange > 0.05 || actualDegreeChange < -0.05) {
            return actualDegreeChange;
          } else {
            return 0;
        }
      }

    private Rotation2d getFromHeading() {
        return Rotation2d.fromDegrees(canCoder.getAbsolutePosition());
    }

}