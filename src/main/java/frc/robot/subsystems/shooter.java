package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class shooter extends SubsystemBase{
    // Flywheel motors
    public final CANSparkMax m_FLflywheel;
    public final CANSparkMax m_FRflywheel;
    public final CANSparkMax m_BLflywheel;
    public final CANSparkMax m_BRflywheel;

    public final double power = 1;

    public shooter() {
        // Motors
        m_FLflywheel = new CANSparkMax(Constants.frontLeftFlywheelMotor, MotorType.kBrushless);
        m_FRflywheel = new CANSparkMax(Constants.frontRightFlywheelMotor, MotorType.kBrushless);
        m_BLflywheel = new CANSparkMax(Constants.backLeftFlywheelMotor, MotorType.kBrushless);
        m_BRflywheel = new CANSparkMax(Constants.backRightFlywheelMotor, MotorType.kBrushless);

        // Set direction
        m_FLflywheel.setInverted(false);
        m_FRflywheel.setInverted(true);
        m_BLflywheel.setInverted(false);
        m_BRflywheel.setInverted(true);
    }

    @Override
    public void periodic() {
        // Update the power
        SmartDashboard.putNumber("Shooter Value", m_FLflywheel.get());
    }

    // TeleOp shooting
    public void shoot() {
        m_FLflywheel.set(power);
        m_FRflywheel.set(power);
        m_BLflywheel.set(power);
        m_BRflywheel.set(power);
    }

    // Stop completely
    public void stop() {
        m_FLflywheel.set(0);
        m_FRflywheel.set(0);
        m_BLflywheel.set(0);
        m_BRflywheel.set(0);
    }

    // Autonomous shooting
    public void autoShoot(double power) {
        m_FLflywheel.set(power);
        m_FRflywheel.set(power);
        m_BLflywheel.set(power);
        m_BRflywheel.set(power);
    }
}
