package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShuffleboardContent;

public class TraySubsystem extends SubsystemBase {

  public double m_TargetRadians;

  private final int m_canId = 57;

  private final WPI_TalonSRX m_driveMotor;

  private final double trayUpPosition = 600;
  private final double trayDownPosition = 1400;
  private final double coralPlacePosition = 2500;

  public TraySubsystem() {
    // Drive Motor setup
    m_driveMotor = new WPI_TalonSRX(m_canId);
    m_driveMotor.configFactoryDefault();
    m_driveMotor.setNeutralMode(NeutralMode.Brake);

    // PID Setup
    m_driveMotor.config_kP(0, 1);
    m_driveMotor.config_kI(0, 0);
    m_driveMotor.config_kD(0, 0);
    m_driveMotor.config_kF(0, 0);

    // Encoder setup
    m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    m_driveMotor.setInverted(false);
    m_driveMotor.setSensorPhase(false);
    m_driveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 30);
    m_driveMotor.configPeakOutputForward(0.3);
    m_driveMotor.configPeakOutputReverse(-1);
    
    m_TargetRadians = m_driveMotor.getSelectedSensorPosition();

    ShuffleboardContent.initTray(this);
  }

  @Override
  public void periodic() {
  }

  public void moveToUpPosition() {
    m_TargetRadians = trayUpPosition;
    setReferencePeriodic();
  }

  public void moveToDownPosition() {
    m_TargetRadians = trayDownPosition;
    setReferencePeriodic();
  }

  public void moveToCoralPlacePosition() {
    m_TargetRadians = coralPlacePosition;
    setReferencePeriodic();
  }

  public double getUpPosition() {
    return trayUpPosition;
  }

  public double getCoralPlacePosition() {
    return coralPlacePosition;
  }

  public void driveTray(double degrees) {
    m_TargetRadians += degrees * 10;
    setReferencePeriodic();
  }

  public double getActualDegrees() {
    return m_driveMotor.getSelectedSensorPosition();
  }

  public double getTargetDegrees() {
    return m_TargetRadians;
  }

  private void setReferencePeriodic() {
    m_TargetRadians = MathUtil.clamp(m_TargetRadians, trayUpPosition, coralPlacePosition);
    m_driveMotor.set(ControlMode.Position, m_TargetRadians);
  }
}