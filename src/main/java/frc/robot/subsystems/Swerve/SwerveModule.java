package frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

    // motors and encoders
    public final CANSparkMax driveMotor;
    public final CANSparkMax turnMotor;
    public final RelativeEncoder driveEncoder;
    public final RelativeEncoder turnEncoder;

    public final PIDController turningPIDController;

    // aBSOLUTE ENCODER
    public final AnalogInput absoluteEncoder;
    public final boolean absoluteEncoderReversed;
    public final double absoluteEncoderOffset;

    public SwerveModule(
        int drivePort, int turnPort, boolean driveReversed, boolean turnReversed, int absoluteEncoderPort, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

            // Motors
            driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
            turnMotor = new CANSparkMax(turnPort, MotorType.kBrushless);
            driveMotor.setInverted(driveReversed);
            turnMotor.setInverted(turnReversed);

            // Encoders
            driveEncoder = driveMotor.getEncoder();
            turnEncoder = turnMotor.getEncoder();

            // Conversions
            driveEncoder.setPositionConversionFactor(Constants.kDriveEncoderRot2Meter);
            driveEncoder.setVelocityConversionFactor(Constants.kDriveEncoderRPM2MeterPerSec);
            turnEncoder.setPositionConversionFactor(Constants.kTurningEncoderRot2Rad);
            turnEncoder.setVelocityConversionFactor(Constants.kTurningEncoderRPM2RadPerSec);

            // Absolute Encoder
            absoluteEncoder = new AnalogInput(absoluteEncoderPort);
            this.absoluteEncoderReversed = absoluteEncoderReversed;
            this.absoluteEncoderOffset = absoluteEncoderOffset;

            //PID Controller
            turningPIDController = new PIDController(0.5, 0, 0);
            turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

            // Reset all position to 0
            driveEncoder.setPosition(0);
            turnEncoder.setPosition(getAbsoluteEncoderRad());

    }

    // get current angle in radians
    public double getAbsoluteEncoderRad() {
        // Voltage applied over max voltage returns the percentage of a rotation
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        // convert to radians
        angle *= 2.0 * Math.PI;
        
        angle -= absoluteEncoderOffset;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    // Return all data of the position of the robot
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition()));
    }

    // Move
    public void setDesiredState(SwerveModuleState state) {
        // Don't move back to 0 after moving
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        // Optimize angle (turn no more than 90 degrees)
        state = SwerveModuleState.optimize(state, getState().angle); 
        // Set power
        driveMotor.set(state.speedMetersPerSecond / Constants.kPhysicalMaxSpeedMetersPerSecond);
        turnMotor.set(turningPIDController.calculate(turnEncoder.getPosition(), state.angle.getRadians()));

        // Telemetry
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());

    }

    // Stop moving
    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}
