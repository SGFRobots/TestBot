// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static final). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static long startTime;
    // Motor Ports
    public static final class MotorPorts {
        // Wheels
        public static final int leftMotorPort = 2;
        public static final int leftFollowerMotorPort = 3;
        public static final int rightMotorPort = 1;
        public static final int rightFollowerMotorPort = 4;

        // Intake
        public static final int bottomIntakeRollerMotor = 2;
        public static final int intakeBeltMotor = 3;
        public static final int topIntakeRollerMotor = 4;
        
        // FlyWheels
        public static final int backLeftFlywheelMotor = 7;
        public static final int frontRightFlywheelMotor = 5;
        public static final int frontLeftFlywheelMotor = 6;
        public static final int backRightFlywheelMotor = 8;
        public static final int spinnerMotor = 10;
        
        // Hook
        public static final int telescopicArmLeftMotor = 1;
        public static final int telescopicArmRightMotor = 9;
        
    }
    
    public static final class Controllers {
        // Controller 1
        public static final int buttonDPort = 4;
        public static final int buttonAPort = 3;
        public static final int buttonGPort = 5;
        public static final int buttonHPort = 6;
        public static final int switchFPort = 2;
        public static final int switchCPort = 7;
        public static final int driveAxis = 1;//Y-Stick Left
        public static final int turnAxis = 3;//X-Stick Right
        public static final int intakeAxis = 4;//Y-Stick Right
        
        // Controller 2
        public static final int buttonBPort2 = 2;
        public static final int buttonYPort2 = 4;
        public static final int buttonLeftBumperPort2 = 5;
        public static final int LHookAxisPort2 = 1;
        public static final int RHookAxisPort2 = 5;

        // Utility
        public static final int GenericHIDPort = 0;
        public static final int GenericHIDPort2 = 1;
        public static final int powerPanelModule = 1;
    }

    // Speeds
    public static final class Speeds {
        public static final double spinnerSpeed = 0.15;
        public static final double shooterSpeakerSpeed = 1;
        public static final double shooterBoostSpeed = 0.5;
        public static final double intakeSpeed = 1;
        public static final double drivetrainAutoSpeed = 0.65;
    }

    
    // LED
    public static final class LED {
        public static final int LEDChannel = 1;
        // LED State PWM values
        public static final double idleLED = -0.21;
        public static final double ringLED = 0.07;
        public static final double speakerLED = -0.97;
        public static final double ampLED = -0.92;
    }

    public static final class Hook {
        //Hook's Proportion-Integral-Derivative(PID) Constants
        public static final double kP = 0.1; 
        public static final double kI = 0;
        public static final double kD = 0; 
        public static final double kIz = 0; 
        public static final double kFF = 0; 
        public static final double kMaxOutput = 1; 
        public static final double kMinOutput = -1;
    
        // Limits
        public static final int maxHookL = 73;
        public static final int maxHookR = 76;

    }
    
    // Slew Values
    public static final double driveSlewRateLimit = 3;
    public static final double turnSlewRateLimit = 10;

    
    public static final class Swerve {
        
        public static final class MotorPorts {
            public static final int kFrontLeftDriveMotorPort = 8;
            public static final int kBackLeftDriveMotorPort = 2;
            public static final int kFrontRightDriveMotorPort = 6;
            public static final int kBackRightDriveMotorPort = 4;
        
            public static final int kFrontLeftTurningMotorPort = 7;
            public static final int kBackLeftTurningMotorPort = 1;
            public static final int kFrontRightTurningMotorPort = 5;
            public static final int kBackRightTurningMotorPort = 3;
            
            public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
            public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
            public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
            public static final int kBackRightDriveAbsoluteEncoderPort = 3;
        }
    
        public static final class Reversed {
            public static final boolean kFrontLeftTurningEncoderReversed = true;
            public static final boolean kBackLeftTurningEncoderReversed = true;
            public static final boolean kFrontRightTurningEncoderReversed = true;
            public static final boolean kBackRightTurningEncoderReversed = true;
        
            public static final boolean kFrontLeftDriveEncoderReversed = true;
            public static final boolean kBackLeftDriveEncoderReversed = true;
            public static final boolean kFrontRightDriveEncoderReversed = false;
            public static final boolean kBackRightDriveEncoderReversed = false;
        
            public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
            public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
            public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
            public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;
        }
    
        
        public static final class Mechanical {
            public static final double kWheelRadius = .0508;
            public static final double kWheelDiameterMeters = 0.1016;
            public static final double kDriveMotorGearRatio= 6.12;
            public static final double kTurningMotorGearRatio =150/7;
            public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
            public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
            public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
            public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
            
            public static final double kTrackWidth = 0.64;
            // Distance between right and left wheels
            public static final double kWheelBase = 0.64;

            public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.254;
            public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.252;
            public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.816;
            public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -4.811;
        
            // Distance between front and back wheels
            public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                    new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                    new Translation2d(-kWheelBase / 2, kTrackWidth / 2));
        
            public static final double kDeadzone = 0.05;
        
            public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
            public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond / kTrackWidth / 2;
        
            public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
            public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
            public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
            public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
        
            public static final double kDriveEncoderResolution = 2048;
            public static final double kTurningEncoderResolution = 42;
            public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        }
    }


}

