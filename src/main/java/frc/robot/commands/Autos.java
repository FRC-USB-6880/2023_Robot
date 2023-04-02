// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.PositionConstants;
import frc.robot.Constants.PositionConstants.ChargingStation;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Slider;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase ScoreConeLevel3(
    DriveTrain m_drive, Elevator m_elevator, Slider m_slider, 
    IntakeArm m_arm, IntakeWheels m_wheels) {
        return new SequentialCommandGroup(
          new ParallelCommandGroup(m_elevator.setHeight(PositionConstants.ScoreConeLevel3.kElevatorHeight),
                    m_slider.slideToLength(PositionConstants.ScoreConeLevel3.kSliderExtension),
                    m_arm.rotateToAngle(PositionConstants.ScoreConeLevel3.kArmAngle)),
          m_wheels.releaseCone());
  }

  public static CommandBase setToStartingPosition(
    Elevator m_elevator, Slider m_slider, 
    IntakeArm m_arm) {
      return new ParallelCommandGroup(
        m_elevator.setHeight(PositionConstants.StartingPosition.kElevatorHeight),
        m_slider.slideToLength(PositionConstants.StartingPosition.kSliderExtension),
        m_arm.rotateToAngle(PositionConstants.StartingPosition.kArmAngle)
      );
  }

  public static CommandBase ScoreAndEngage(
    DriveTrain m_drive, Elevator m_elevator, Slider m_slider, 
    IntakeArm m_arm, IntakeWheels m_wheels) {
      return new SequentialCommandGroup(
        ScoreConeLevel3(m_drive, m_elevator, m_slider, m_arm, m_wheels),
        setToStartingPosition(m_elevator, m_slider, m_arm),
        m_drive.driveToChargeStationCmd(0.3)
      );
  }

  public static CommandBase ScoreAndMobilityShort(
    DriveTrain m_drive, Elevator m_elevator, Slider m_slider, 
    IntakeArm m_arm, IntakeWheels m_wheels) {
      return new SequentialCommandGroup(
        ScoreConeLevel3(m_drive, m_elevator, m_slider, m_arm, m_wheels),
        setToStartingPosition(m_elevator, m_slider, m_arm),
        m_drive.driveDistanceCommand(PositionConstants.ChargingStation.communityShortDist, 0.3)
      );
  }

  public static CommandBase ScoreAndMobilityLong(
    DriveTrain m_drive, Elevator m_elevator, Slider m_slider, 
    IntakeArm m_arm, IntakeWheels m_wheels) {
      return new SequentialCommandGroup(
        ScoreConeLevel3(m_drive, m_elevator, m_slider, m_arm, m_wheels),
        setToStartingPosition(m_elevator, m_slider, m_arm),
        m_drive.driveDistanceCommand(ChargingStation.communityLongDist, 0.3)
      );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
