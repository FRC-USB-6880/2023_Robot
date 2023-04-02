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
import frc.robot.Constants.SliderConstants;

public class Slider extends SubsystemBase {
  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  /** Creates a new Slider. */
  public Slider() {
    m_motor = new CANSparkMax(CAN_IDs.slider, MotorType.kBrushless);
    m_motor.setInverted(SliderConstants.kMotorInverted);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSmartCurrentLimit(SliderConstants.kCurrentLimit);
    m_motor.burnFlash();

    m_encoder = m_motor.getEncoder();
    m_encoder.setPositionConversionFactor(SliderConstants.kPositionFactor);
    m_encoder.setVelocityConversionFactor(SliderConstants.kVelocityFactor);
    resetEncoder();

    SmartDashboard.putData(this);
  }

  private void resetEncoder() {
    m_encoder.setPosition(0);
  }
  private boolean sliderIsNotSafe() {
    return m_motor.getOutputCurrent() > SliderConstants.kCurrentLimit;
  }

  public CommandBase stopCmd() {
    return this.runOnce(() -> m_motor.set(0))
              .withName("Slider Stop Command");
  }

  private boolean sliderCanSlideOut() {
    return (m_encoder.getPosition() < SliderConstants.kMaxTravelInInches);
  }

  private boolean sliderCanSlideIn() {
    return (m_encoder.getPosition() > SliderConstants.kMinTravelInInches);
  }

  private boolean sliderIsAtExtension(double lengthInches) {
    return Math.abs(m_encoder.getPosition() - lengthInches) < SliderConstants.kToleranceInInches;
  }

  private boolean sliderExtensionIsMoreThan(double length) {
    return (m_encoder.getPosition() >= length);
  }

  private boolean sliderExtensionIsLessThan(double length) {
    return (m_encoder.getPosition() <= length);
  }

  public CommandBase slideOutToLength(double length) {
    return this.run(() -> m_motor.set(SliderConstants.kSliderSpeedOut))
              .unless(() -> sliderExtensionIsMoreThan(length))
              .until(() -> sliderIsAtExtension(length) ||
                          sliderExtensionIsMoreThan(length) ||
                          sliderIsNotSafe() ||
                          !sliderCanSlideOut())
              .finallyDo((interrupted) -> m_motor.set(0))
              .withName("slideOutToLength");
  }

  public CommandBase slideInToLength(double length) {
    return this.run(() -> m_motor.set(SliderConstants.kSliderSpeedIn))
              .unless(() -> sliderExtensionIsLessThan(length))
              .until(() -> sliderIsAtExtension(length) ||
                          sliderExtensionIsLessThan(length) ||
                          sliderIsNotSafe() ||
                          !sliderCanSlideIn())
              .finallyDo((interrupted) -> m_motor.set(0))
              .withName("slideInToLength");
  }

  public CommandBase slideToLength(double length) {
    return new ConditionalCommand(slideOutToLength(length), slideInToLength(length), () -> sliderExtensionIsLessThan(length))
                  .unless(() -> sliderIsNotSafe())
                  ;
  }

  public CommandBase slideOut() {
    return this.runOnce(() -> m_motor.set(SliderConstants.kSliderSpeedOut))
              .unless(() -> !sliderCanSlideOut());
  }

  public CommandBase slideIn() {
    return this.runOnce(() -> m_motor.set(SliderConstants.kSliderSpeedIn))
              .unless(() -> !sliderCanSlideIn());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Slider Position", m_encoder.getPosition());
    SmartDashboard.putBoolean("Slider Can slide out", sliderCanSlideOut());
    SmartDashboard.putBoolean("Slider can slide In", sliderCanSlideIn());
    SmartDashboard.putNumber("Current Output", m_motor.getOutputCurrent());
  }
}
