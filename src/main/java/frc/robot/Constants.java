// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class CAN_IDs {
    public static final int driveTrain_Left1 = 11;
    public static final int driveTrain_Left2 = 12;
    public static final int driveTrain_Right1 = 13;
    public static final int driveTrain_Right2 = 14;
    public static final int elevatorLeft = 21;
    public static final int elevatorRight = 22;
    public static final int slider = 23;
    public static final int intakeArmLeft = 26;
    // public static final int intakeArmRight = 27;
    public static final int intakeWheels = 28;
  }

  public static class DriveTrainConstants {
    public static final boolean kLeftInverted = true;
    public static final boolean kRightInverted = false;

    public static final int kCurrentLimit = 60;

    public static final double kTurningScale = 0.5;

    public static final double kWheelDiameter = 6.0; // in inches
    public static final double _gearRaio = (11/52) * (30/68); // == 0.0933 = 1/10.715
    // kPositionFactor converts # of motor rotations to distance travelled
    public static final double kPositionFactor = _gearRaio * Math.PI * kWheelDiameter;
    // kVelocityFactor converts # of motor RPM to inches/sec
    public static final double kVelocityFactor = kPositionFactor / 60;
  }

  public static class ElevatorConstants {
    public static final boolean kMotorLeft_inverted = false;
    public static final boolean kMotorRight_inverted = true;
    public static final int kCurrentLimit = 80;
    public static final double kElevatorGearRatio = 1 / (5 * 4); // 5:1 * 4:1
    // RevRobotics 16T Sprocket pitch size is 1.29 inches
    // https://www.revrobotics.com/25-sprockets/
    // https://www.revrobotics.com/content/docs/REV-21-2016-DR.pdf
    // kPositionFactor converts #motor-rotations into #inches travelled by the Elevator
    public static final double kPositionFactor = Math.PI * 1.29 * kElevatorGearRatio;
    // kVelocityFactor converts Motor-RPM (Revolutions Per Minute) into  inches-per-second
    public static final double kVelocityFactor = kPositionFactor / 60;

    public static final double kMinTravelInInches = 0;
    public static final double kMaxTravelInInches = 36; // ToDo:  Verify with experimentation
    public static final double kToleranceInInches = 1.8; // 5% of travel length
    public static final double kElevatorSpeedUp = 0.5;
    public static final double kElevatorSpeedDown = -0.3;
  }

  public static class SliderConstants {
    public static final boolean kMotorInverted = false;
    public static final int kCurrentLimit = 40;
    public static final double kSliderGearRatio = 1 / (4 * 3); // 4:1 * 3:1 MaxPlanetary gearbox
    // RevRobotics 16T Sprocket pitch size is 1.29 inches
    // https://www.revrobotics.com/25-sprockets/
    // https://www.revrobotics.com/content/docs/REV-21-2016-DR.pdf
    // kPositionFactor converts #motor-rotations into #inches travelled by the Elevator
    public static final double kPositionFactor = Math.PI * 1.29 * kSliderGearRatio;
    // kVelocityFactor converts Motor-RPM (Revolutions Per Minute) into  inches-per-second
    public static final double kVelocityFactor = kPositionFactor / 60;

    public static final double kMinTravelInInches = 0;
    public static final double kMaxTravelInInches = 23; // ToDo:  Verify with experimentation
    public static final double kToleranceInInches = 1.2; // 5% of travel length

    public static final double kSliderSpeedOut = 0.3;
    public static final double kSliderSpeedIn = -0.3;
  }

  public static class IntakeArmConstants {
    public static final boolean kMotorLeftInverted = false;
    public static final boolean kMotorRightInverted = true;
    public static final int kCurrentLimit = 80;
    // We want 1 rotations of motor = 6 degrees of arm
    // => 1 m = 6/360 a => a/m = 360/6 = 60
    public static final double kArmGearRatio = 1 / (5 * 4 * 4); // 5:1 * 4:1 * 64T:16T
    // kPostionFactor converts #MotorRotations to #ArmRadians
    public static final double kPositionFactor = Units.rotationsToRadians(kArmGearRatio);
    // kVelocityFactor converts #MotorRevolutionsPerMinute to #ArmRadianPerSecond
    public static final double kVelocityFactor = Units.rotationsPerMinuteToRadiansPerSecond(kArmGearRatio);
    public static final double kArmFreeSpeed = 5676.0 * kVelocityFactor;
    public static final double kArmUpSpeed = 0.7;
    public static final double kArmDownSpeed = -0.3;
    public static final double kArmLowerLimitDeg = 45; // in degrees; 0 degres => competely folded
    public static final double kArmUpperLimitDeg = 300;  // in degrees; 0 degres => competely folded
    // Convert the above degrees values to # of Radians
    public static final double kArmLowerLimit = Units.degreesToRadians(kArmLowerLimitDeg);
    public static final double kArmUpperLimit = Units.degreesToRadians(kArmUpperLimitDeg);
    public static final double kToleranceRadians = Units.degreesToRadians(5); // 5 degree tolerance
    public static final double kArmPickupFromLoadingStation = 0.3; // To be changed after testing

  }
  public static class IntakeWheelsConstants {
    public static final boolean kWheelsInverted = false;
    public static final int kCurrentLimit = 70;
    // The current being drawn goes up when the cone or cube ir grabbed.
    // In the constant definitions below, we are assuming that current drawn is >= 20amp
    //    when a cone is acquired by the intake wheels.
    public static final double kCurrentWhenConeIsGrabbed = 20;
    public static final double kCurrentWhenCubeIsGrabbed = 20;
    public static final double kConeGrabSpeed = 0.9;
    public static final double kConeReleaseSpeed = -0.5;
    public static final double kCubeGrabSpeed = 0.8;
    public static final double kCubeReleaseSpeed = -0.5;

  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort1 = 0;
    public static final int kDriverControllerPort2 = 1;
    public static final double kDriveDeadband = 0.05;
    public static final double kArmManualDeadband = 0.05;
    public static final double kArmManualScale = 0.5;
  }

  public static class PositionConstants {
    public static class StartingPosition {
      public static final double kElevatorHeight = 0.0; // inches
      public static final double kSliderExtension = 0.0; // inches
      public static final double kArmAngle = 30; // degrees
    }
    public static class ScoreConeLevel3 {
      public static final double kElevatorHeight = 36.0; // inches
      public static final double kSliderExtension = 36.0; // inches
      public static final double kArmAngle = 135; // degrees
    }
    public static class ChargingStation {
      public static final double lowerRampDegrees = 11;
      public static final double upperRampDegrees = 71.5;
      // Note: When the charge station is level, both the ramps on 
      // either side of the main surface are at 15 degrees.
      // But this number is irrelevant for the purpose autonomous programming.
      public static final double RampDegreesWhenEngaged = 34.25;
      public static final double levelTolerance = 2.5; // in degrees
      public static final double height = 9.125; // in inches
      public static final double heightTolerance = 2; // in inches
      public static final double communityShortDist = (11*12)+(3/8); // in inches
      public static final double communityLongDist = (16*12) + 1.25; // in inches
    }
  }
}
