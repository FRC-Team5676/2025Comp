package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShuffleboardContent;

public class TraySubsystem extends SubsystemBase {

  public double m_TargetRadians;

  private final int m_canId = 56;

  private final WPI_TalonSRX m_driveMotor;

  private final double trayUpPosition = Units.degreesToRadians(-300);
  private final double trayDownPosition = Units.degreesToRadians(-300);

  public TraySubsystem() {
    // Drive Motor setup
    m_driveMotor = new WPI_TalonSRX(m_canId);
    m_driveMotor.configFactoryDefault();
    m_driveMotor.setNeutralMode(NeutralMode.Brake);

    // PID Setup
    m_driveMotor.config_kP(0, 0.04);
    m_driveMotor.config_kI(0, 0);
    m_driveMotor.config_kD(0, 0);
    m_driveMotor.config_kF(0, 0);

    // Encoder setup
    m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    m_driveMotor.setInverted(true);
    m_driveMotor.setSensorPhase(true);
    m_driveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 30);
    m_driveMotor.configPeakOutputForward(1);
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

  public double getUpPosition() {
    return trayUpPosition;
  }

  public double getDownPosition() {
    return trayDownPosition;
  }

  public double getActualDegrees() {
    return Units.radiansToDegrees(m_driveMotor.getSelectedSensorPosition());
  }

  public double getTargetDegrees() {
    return Units.radiansToDegrees(m_TargetRadians);
  }

  private void setReferencePeriodic() {
    m_TargetRadians = MathUtil.clamp(m_TargetRadians, trayUpPosition, trayDownPosition);
    m_driveMotor.set(ControlMode.Position, m_TargetRadians);
  }
}