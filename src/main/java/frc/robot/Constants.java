// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotorPrimaryPort = 4;
    public static final int kLeftMotorSecondaryPort = 17;
    public static final int kRightMotorPrimaryPort = 7;
    public static final int kRightMotorSecondaryPort = 18;
    public static final int kTurnTravelUnitsPerRotation = 3600;
    public static final int kEncoderUnitsPerRotation = 51711; // number is added by experimentation
    public final static double kNeutralDeadband = 0.001;
    public final static Gains kGains_Turning = new Gains(1.5, 0.0, 4.0, 0.0, 200, 1.00);
    public final static int kTimeoutMs = 30;
    public final static int PID_PRIMARY = 0;
    public final static int REMOTE_0 = 0;
    public final static int REMOTE_1 = 1;
    public final static int PID_TURN = 1;
    public final static int SLOT_1 = 1;
    public final static int kSlot_Turning = SLOT_1;
  }

  public static final class ElevatorConstants {
    public static final int kElevMotorPort = 10;

    public static final Gains kElevGains = new Gains(0.15, 0.0, 0.0, 0.0, 200, 1.00);
    public static final int kElevEncoderRotationsAtMaxHeight = -2004016; // TODO: determine emperically
    public static final int elevatorSafeWenchPosition = -332606;
  }

  public static final class UpperWenchConstants {
    public static final int kUpperMotorPort = 29;

    public static final Gains kUpperGains = new Gains(0.15, 0.0, 0.0, 0.0, 200, 1.00);
    public static final int kUpeprEncoderRotationsAtMaxHeight = -2004016; // TODO: determine emperically
    public static final int UpperWenchSafeWenchPosition = -332606;
  }

  public static final class WenchConstants {
    public static final int kWenchPrimaryMotorPort = 21;
    public static final int kWenchSecondaryMotorPort = 20;
    public static final Gains kWenchGains = new Gains(0.15, 0.0, 0.0, 0.0, 200, 0.2);

  }

  public static final class ClawConstants {
    public static final int kClawMotorPort = 8;
    public static final Gains kClawGains = new Gains(0.15, 0.0, 0.0, 0.0, 200, 1.00);
    public static final int kClawEncoderRotationsAtMaxExtension = 42000;
    public static final int kClawCubePosition = 20000;
    public static final int kClawConePosition = 3500;

  }

  public static final class LightingConstants {
  }

  public static final class AutoConstants {
    public static final double kShortDriveTimeSeconds = 2.5;
    public static final double kLongDriveTimeSeconds = 7;
    public static final double kDriveSpeed = 0.6;

  }

  public static final class IOConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;

    // Buttons are 1-indexed
    public static final int kCoDriverButtonA = 1;
    public static final int kCoDriverButtonB = 2;
    public static final int kCoDriverButtonX = 3;
    public static final int kCoDriverButtonY = 4;
    public static final int kCoDriverButtonLeftBumber = 5;
    public static final int kCoDriverButtonRightBumber = 6;
    public static final int kCoDriverButtonBack = 7;
    public static final int kCoDriverButtonStart = 8;
    public static final int kCoDriverButtonLeftJoystick = 9;
    public static final int kCoDriverButtonRightJoystick = 10;
    public static final int kCoDriverAxisRightTrigger = 3;
  }
}