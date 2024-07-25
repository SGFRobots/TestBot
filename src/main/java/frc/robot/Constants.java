// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Motor Ports
    public static int leftMotorPort = 2;
    public static int leftFollowerMotorPort = 3;
    public static int rightMotorPort = 1;
    public static int rightFollowerMotorPort = 4;

    // Speeds
    public static double spinnerSpeed = 0.15;
    public static double shooterSpeakerSpeed = 1;
    public static double shooterBoostSpeed = 0.5;
    public static double intakeSpeed = 1;
    public static double drivetrainAutoSpeed = 0.65;

    // Intake Port Constants
    public static int bottomIntakeRollerMotor = 2;
    public static int intakeBeltMotor = 3;
    public static int topIntakeRollerMotor = 4;

    // Flywheel Port Constants
    public static int backLeftFlywheelMotor = 7;
    public static int frontRightFlywheelMotor = 5;
    public static int frontLeftFlywheelMotor = 6;
    public static int backRightFlywheelMotor = 8;
    public static int spinnerMotor = 10;

    // Hook Port Constants
    public static int telescopicArmLeftMotor = 1;
    public static int telescopicArmRightMotor = 9;

    // Utility Port Constants
    public static int GenericHIDPort = 0;
    public static int GenericHIDPort2 = 1;
    public static int powerPanelModule = 1;

    // First Controller Ports
    public static int buttonDPort = 4;
    public static int buttonAPort = 3;
    public static int buttonGPort = 5;
    public static int buttonHPort = 6;
    public static int switchFPort = 2;
    public static int switchCPort = 7;
    public static int driveAxis = 1;//Y-Stick Left
    public static int turnAxis = 3;//X-Stick Right
    public static int intakeAxis = 4;//Y-Stick Right
    
    // Second Controller Ports
    public static int buttonBPort2 = 2;
    public static int buttonYPort2 = 4;
    public static int buttonLeftBumperPort2 = 5;
    public static int LHookAxisPort2 = 1;
    public static int RHookAxisPort2 = 5;

    // LED
    public static int LEDChannel = 1;
    // LED State PWM values
    public static double idleLED = -0.21;
    public static double ringLED = 0.07;
    public static double speakerLED = -0.97;
    public static double ampLED = -0.92;

    // Slew Values
    public static double driveSlewRateLimit = 3;
    public static double turnSlewRateLimit = 10;

    //Hook's Proportion-Integral-Derivative(PID) Constants
    public static double kP = 0.1; 
    public static double kI = 0;
    public static double kD = 0; 
    public static double kIz = 0; 
    public static double kFF = 0; 
    public static double kMaxOutput = 1; 
    public static double kMinOutput = -1;

    //Autonomous Time
    public static long startTime;

    // Limits
    public static int maxHookL = 73;
    public static int maxHookR = 76;
    //Autos
    
    //Swerve
    public static final double kWheelRadius = .0508;
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kDriveMotorGearRatio= 6.12;
    public static final double kTurningMotorGearRatio =150/7;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

    public static final int kFrontLeftDriveMotorPort = 8;
    public static final int kBackLeftDriveMotorPort = 2;
    public static final int kFrontRightDriveMotorPort = 6;
    public static final int kBackRightDriveMotorPort = 4;

    public static final int kFrontLeftTurningMotorPort = 7;
    public static final int kBackLeftTurningMotorPort = 1;
    public static final int kFrontRightTurningMotorPort = 5;
    public static final int kBackRightTurningMotorPort = 3;

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
    public static final int kBackRightDriveAbsoluteEncoderPort = 3;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.254;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.252;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.816;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -4.811;

    public static final double kTrackWidth = 0.64;
    // Distance between right and left wheels
    public static final double kWheelBase = 0.64;
    // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    public static final double kDeadzone = 0.05;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond / kWheelDiameterMeters / 2;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final double kDriveEncoderResolution = 2048;
    public static final double kTurningEncoderResolution = 42;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;


}
