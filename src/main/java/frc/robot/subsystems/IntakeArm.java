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
import frc.robot.Constants.IntakeArmConstants;

public class IntakeArm extends SubsystemBase {
  private CANSparkMax m_motorLeft, m_motorRight;
  private RelativeEncoder m_encoder;

  /** Creates a new ExampleSubsystem. */
  public IntakeArm() {
    m_motorLeft = new CANSparkMax(CAN_IDs.intakeArmLeft, MotorType.kBrushless);
    m_motorLeft.setInverted(IntakeArmConstants.kMotorLeftInverted);
    m_motorLeft.setIdleMode(IdleMode.kBrake);
    m_motorLeft.setSmartCurrentLimit(IntakeArmConstants.kCurrentLimit);
    m_motorLeft.burnFlash();

    m_motorRight = new CANSparkMax(CAN_IDs.intakeArmRight, MotorType.kBrushless);
    m_motorRight.setIdleMode(IdleMode.kBrake);
    m_motorRight.setSmartCurrentLimit(IntakeArmConstants.kCurrentLimit);
    m_motorRight.follow(m_motorLeft, IntakeArmConstants.kMotorRightInverted);
    m_motorRight.burnFlash();

    m_encoder = m_motorLeft.getEncoder();
    m_encoder.setPositionConversionFactor(IntakeArmConstants.kPositionFactor);
    m_encoder.setVelocityConversionFactor(IntakeArmConstants.kVelocityFactor);
    // The starting position is set as 0 in the constructor.
    // This should be done only once for the Arm!
    m_encoder.setPosition(0);

    SmartDashboard.putData(this);
  }

  private double getCurAngle() {
    return m_encoder.getPosition();
  }

  private boolean armIsAtAngle(double armRadians) {
    return (Math.abs(getCurAngle() - armRadians) < IntakeArmConstants.kToleranceRadians);
  }

  private boolean armIsBelowAngle(double armRadians) {
    return (getCurAngle() <= armRadians);
  }

  private boolean armIsAboveAngle(double armRadians) {
    return (getCurAngle() >= armRadians);
  }

  /**
   * rotateClockwise command factory method.
   *
   * @return a command
   */
  public CommandBase rotateToAngleClockwise(double armRadians) {
    // Inline construction of command goes here.
    // Subsystem::run implicitly requires `this` subsystem.
    return this.run(() -> m_motorLeft.set(IntakeArmConstants.kArmUpSpeed))
                .unless(() -> armIsAboveAngle(armRadians)) // Arm is already above the given angle
                .until(() -> armIsAtAngle(armRadians) || armIsAboveAngle(armRadians) || 
                              armIsNotsafe() || !armHasRoomFor_CW_rotation())
                .finallyDo((interrupted) -> stop())
                .withName("Rotate Up (Clockwise) To Angle");
  }

  public CommandBase rotateToAngleCounterClockwise(double armRadians) {
    return this.run(() -> m_motorLeft.set(IntakeArmConstants.kArmDownSpeed))
                .unless(() -> armIsBelowAngle(armRadians)) // Arm is already below the given angle
                .until(() -> armIsBelowAngle(armRadians) || armIsBelowAngle(armRadians) || 
                              armIsNotsafe() || !armHasRoomFor_CCW_rotation())
                .finallyDo((interrupted) -> stop())
                .withName("Rotate Down (Counter Clockwise) to Angle");
  }

  public CommandBase rotateToAngle(double armRadians) {
    return new ConditionalCommand(rotateToAngleClockwise(armRadians), 
                                  rotateToAngleCounterClockwise(armRadians), 
                                  () -> armIsBelowAngle(armRadians))
                .unless(() -> armIsNotsafe())
                .finallyDo((interrupted) -> stop())
                .withName("rotateToAngle");
  }

  /**
   * turnUp command factory method.
   *
   * @return a command
   */
  public CommandBase turnUp() {
    // Inline construction of command goes here.
    // Subsystem::runOnce implicitly requires `this` subsystem.
    return this.runOnce(() -> m_motorLeft.set(IntakeArmConstants.kArmUpSpeed))
            .unless(() -> this.armIsNotsafe() || !armHasRoomFor_CW_rotation());
  }

  /**
   * turnDown command factory method.
   *
   * @return a command
   */
  public CommandBase turnDown() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return this.runOnce(() -> m_motorLeft.set(IntakeArmConstants.kArmDownSpeed))
              .unless(() -> this.armIsNotsafe() || !armHasRoomFor_CCW_rotation());
  }

  private void stop() {
    m_motorLeft.set(0);
  }

  /**
   * stop command factory method.
   *
   * @return a command
   */
  public CommandBase stopCmd() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return this.runOnce(() -> m_motorLeft.set(0.0));
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean armIsNotsafe() {
    return ((m_motorLeft.getOutputCurrent() < IntakeArmConstants.kCurrentLimit) &&
        (m_motorRight.getOutputCurrent() < IntakeArmConstants.kCurrentLimit));
  }

  public boolean armHasRoomFor_CW_rotation() {
    return armIsBelowAngle(IntakeArmConstants.kArmLowerLimit);
  }

  public boolean armHasRoomFor_CCW_rotation() {
    return armIsAboveAngle(IntakeArmConstants.kArmUpperLimit);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Display the current Arm Position on Smart Dashboard
    double curPos = m_encoder.getPosition();
    SmartDashboard.putNumber("Arm Position", curPos);
    SmartDashboard.putBoolean("Arm can move in CW direction", armHasRoomFor_CW_rotation());
    SmartDashboard.putBoolean("Arm can move in CCW direction", armHasRoomFor_CCW_rotation());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    double curPos = m_encoder.getPosition();
    SmartDashboard.putNumber("Arm Position", curPos);
  }
}
