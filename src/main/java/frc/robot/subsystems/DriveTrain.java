// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.SparkMaxRelativeEncoder.Type;

import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.PositionConstants.ChargingStation;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends SubsystemBase {
  private CANSparkMax m_left1, m_left2, m_right1, m_right2;
  private RelativeEncoder m_leftEncoder, m_rightEncoder;
  private AHRS m_gyro;
  private double _altitude;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    m_left1  = new CANSparkMax(CAN_IDs.driveTrain_Left1, MotorType.kBrushless);
    m_left1.setInverted(DriveTrainConstants.kLeftInverted);
    m_left1.setSmartCurrentLimit(DriveTrainConstants.kCurrentLimit);
    m_left1.setIdleMode(IdleMode.kBrake);
    m_left1.burnFlash();

    m_left2  = new CANSparkMax(CAN_IDs.driveTrain_Left2, MotorType.kBrushless);
    m_left2.setInverted(DriveTrainConstants.kLeftInverted);
    m_left2.setSmartCurrentLimit(DriveTrainConstants.kCurrentLimit);
    m_left2.setIdleMode(IdleMode.kBrake);
    m_left2.burnFlash();

    m_right1  = new CANSparkMax(CAN_IDs.driveTrain_Right1, MotorType.kBrushless);
    m_right1.setInverted(DriveTrainConstants.kRightInverted);
    m_right1.setSmartCurrentLimit(DriveTrainConstants.kCurrentLimit);
    m_right1.setIdleMode(IdleMode.kBrake);
    m_right1.burnFlash();

    m_right2  = new CANSparkMax(CAN_IDs.driveTrain_Right2, MotorType.kBrushless);
    m_right2.setInverted(DriveTrainConstants.kRightInverted);
    m_right2.setSmartCurrentLimit(DriveTrainConstants.kCurrentLimit);
    m_right2.setIdleMode(IdleMode.kBrake);
    m_right2.burnFlash();

    m_leftEncoder = m_left1.getEncoder();
    // m_leftEncoder = m_left1.getEncoder(Type.kHallSensor, 42);
    m_leftEncoder.setPositionConversionFactor(DriveTrainConstants.kPositionFactor);
    m_rightEncoder = m_right1.getEncoder();
    // m_rightEncoder = m_left1.getEncoder(Type.kHallSensor, 42);
    m_rightEncoder.setPositionConversionFactor(DriveTrainConstants.kPositionFactor);

    m_gyro = new AHRS(Port.kMXP);
    m_gyro.resetDisplacement();
    m_gyro.reset();
  }

  public void driveArcade(double _straight, double _turn) {
    double left  = MathUtil.clamp(_straight + _turn, -1.0, 1.0);
    double right = MathUtil.clamp(_straight - _turn, -1.0, 1.0);

    m_left1.set(left);
    m_left2.set(left);
    m_right1.set(right);
    m_right2.set(right);
  }

  private void resetGyro() {
    m_gyro.resetDisplacement();
    m_gyro.reset();
    _altitude = 0.0;
  }


  public CommandBase driveToChargeStationCmd(double speed) {
    return this.run(() -> driveArcade(speed, 0))
              .beforeStarting(() -> 
                {
                  this.resetEncoders();
                  this.resetGyro();
                })
              .until(() -> 
                {
                  double theta, driveDist;
                  // Note: theta is in degrees units [-180, 180], whereas Math.sin() takes radians as input
                  theta = m_gyro.getPitch();
                  driveDist = distanceTravelled();
                  _altitude += driveDist * Math.sin(Units.degreesToRadians(theta));
                  // Apply some tolerance to theta
                  // Charging station height is 9.125"
                  // Add distanceTravelled condition also
                  if (Math.abs(_altitude - ChargingStation.height) < ChargingStation.heightTolerance && 
                      Math.abs(theta) <= ChargingStation.levelTolerance) {
                    return true;
                  } else {
                    return false;
                  }
                })
              .withTimeout(10)
              .finallyDo((interrupted) -> driveArcade(0, 0));
  }

  public CommandBase driveTimeCommand(double seconds, double speed) {
    return this.run(() -> driveArcade(speed, 0))
              .withTimeout(seconds)
              .finallyDo((interrupted) -> driveArcade(0, 0));
  }

  /**
   * Returns a command that drives the robot forward a specified distance at a specified speed.
   *
   * @param distanceMeters The distance to drive forward in meters
   * @param speed The fraction of max speed at which to drive
   */
  public CommandBase driveDistanceCommand(double distanceInches, double speed) {
    return this.run(() -> driveArcade(speed, 0)) // Drive forward at specified speed
              .beforeStarting(() -> resetEncoders(), this)
              .until(() -> distanceTravelled() >= distanceInches)
              // Stop the drive when the command ends
              .finallyDo(interrupted -> this.driveArcade(0, 0));
 }

  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public double distanceTravelled() {
    // Assumes that the Robot is moving straight.
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pitch", m_gyro.getPitch());
    SmartDashboard.putNumber("Roll", m_gyro.getRoll());
    SmartDashboard.putNumber("Yaw", m_gyro.getYaw());
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    //builder.addDoubleProperty("Setpoint", () -> m_setpoint, (val) -> m_setpoint = val);
    //builder.addBooleanProperty("At Setpoint", () -> atSetpoint(), null);
    //addChild("Controller", m_controller);
  }
}
