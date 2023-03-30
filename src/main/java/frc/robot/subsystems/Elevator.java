// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private final CANSparkMax m_motorLeft, m_motorRight;
  private final RelativeEncoder m_encoderLeft, m_encoderRight;

  /** Creates a new Elevator. */
  public Elevator() {
    m_motorLeft = new CANSparkMax(CAN_IDs.elevatorLeft, MotorType.kBrushless);
    m_motorLeft.setInverted(ElevatorConstants.kMotorLeft_inverted);
    m_motorLeft.setIdleMode(IdleMode.kBrake);
    m_motorLeft.setSmartCurrentLimit(ElevatorConstants.kCurrentLimit);
    m_motorLeft.burnFlash();

    m_motorRight = new CANSparkMax(CAN_IDs.elevatorRight, MotorType.kBrushless);
    m_motorRight.setIdleMode(IdleMode.kBrake);
    m_motorRight.setSmartCurrentLimit(ElevatorConstants.kCurrentLimit);
    m_motorRight.follow(m_motorLeft, ElevatorConstants.kMotorRight_inverted);
    m_motorRight.burnFlash();

    m_encoderLeft = m_motorLeft.getEncoder();
    m_encoderLeft.setPositionConversionFactor(ElevatorConstants.kPositionFactor);
    m_encoderLeft.setVelocityConversionFactor(ElevatorConstants.kVelocityFactor);
    
    m_encoderRight = m_motorRight.getEncoder();
    m_encoderRight.setPositionConversionFactor(ElevatorConstants.kPositionFactor);
    m_encoderRight.setVelocityConversionFactor(ElevatorConstants.kVelocityFactor);
    resetEncoders();

    SmartDashboard.putData(this);
  }

  private void resetEncoders() {
    m_encoderLeft.setPosition(0);
    m_encoderRight.setPosition(0);
  }

  private void setSpeed(double speed) {
    m_motorLeft.set(speed);
    // m_motorRight was set to follow m_motorLeft
    // m_motorRight.set(speed);
  }

  public CommandBase stopCmd() {
    return this.runOnce(() -> setSpeed(0))
              .withName("Elevator Stop Command");
  }

  public boolean elevatorCanGoLower() {
    double leftPosition = m_encoderLeft.getPosition();
    double rightPosition = m_encoderRight.getPosition();
    return ((leftPosition > ElevatorConstants.kMinTravelInInches) &&
            (rightPosition > ElevatorConstants.kMinTravelInInches));
  }

  public boolean elevatorCanGoHigher() {
    double leftPosition = m_encoderLeft.getPosition();
    double rightPosition = m_encoderRight.getPosition();
    return ((leftPosition < ElevatorConstants.kMaxTravelInInches) &&
            (rightPosition < ElevatorConstants.kMaxTravelInInches));
  }

  public boolean elevatorIsNotSafe() {
    return ((m_motorLeft.getOutputCurrent() > ElevatorConstants.kCurrentLimit) ||
            (m_motorRight.getOutputCurrent() > ElevatorConstants.kCurrentLimit));
  }

  public boolean elevatorIsAtHeight(double height) {
    double curHeight = (m_encoderLeft.getPosition() + m_encoderRight.getPosition()) / 2;
    return (Math.abs(curHeight - height) < ElevatorConstants.kToleranceInInches);
  }

  public CommandBase raise() {
    return this.runOnce(() -> setSpeed(ElevatorConstants.kElevatorSpeedUp))
              .unless(() -> elevatorIsNotSafe() || !elevatorCanGoHigher());
  }

  public CommandBase lower() {
    return this.runOnce(() -> setSpeed(ElevatorConstants.kElevatorSpeedDown))
              .unless(() -> elevatorIsNotSafe() || !elevatorCanGoLower());
  }

  public CommandBase raiseToHeight(double desiredHeight) {
    return this.run(() -> setSpeed(ElevatorConstants.kElevatorSpeedUp))
              .unless(() -> (elevatorIsNotSafe() ||
                      (desiredHeight <= getCurHeight()) ||
                      elevatorIsAtHeight(desiredHeight)))
              .until(() -> elevatorIsAtHeight(desiredHeight))
              .finallyDo((interrupted) -> setSpeed(0))
              .withName("raiseToHeight");
  }

  public CommandBase lowerToHeight(double desiredHeight) {
    return this.run(() -> setSpeed(ElevatorConstants.kElevatorSpeedDown))
              .unless(() -> (elevatorIsNotSafe() || 
                            desiredHeight >= getCurHeight() ||
                            elevatorIsAtHeight(desiredHeight)))
              .until(() -> elevatorIsAtHeight(desiredHeight))
              .finallyDo((interrupted) -> setSpeed(0))
              .withName("lowerToHeight");
  }

  private double getCurHeight() {
    return (m_encoderLeft.getPosition() + m_encoderRight.getPosition()) / 2;
  }

  public CommandBase setHeight(double desiredHeight) {
    return new ConditionalCommand(raiseToHeight(desiredHeight), 
                                  lowerToHeight(desiredHeight), 
                                  () -> (desiredHeight > getCurHeight()))
                .unless(() -> Math.abs(getCurHeight() - desiredHeight) < ElevatorConstants.kToleranceInInches)
                .finallyDo((interrupted) -> setSpeed(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LeftPosition", m_encoderLeft.getPosition());
    SmartDashboard.putNumber("RightPosition", m_encoderRight.getPosition());
    SmartDashboard.putBoolean("Is Elevator in Safe position?", !elevatorIsNotSafe());
    SmartDashboard.putBoolean("ElevatorCanGoHigher", elevatorCanGoHigher());
    SmartDashboard.putBoolean("ElevatorCanGoLower", elevatorCanGoLower());
  }
}
