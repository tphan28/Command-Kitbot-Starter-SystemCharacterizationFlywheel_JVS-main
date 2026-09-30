// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


import static frc.robot.Constants.IOConstants.*;

/**
 * STARTER VERSION — basic open-loop (duty cycle) control only.
 *
 * This is a stripped-down version of the Flywheel subsystem meant for
 * system characterization. There is no PID or feedforward here — you are
 * commanding raw duty cycle (percent output, 0.0 to 1.0) directly to the
 * motor and reading back whatever velocity/voltage/current results.
 *
 * Two ways to characterize are available:
 *   - By hand: step through a few duty cycle values with the increase/decrease
 *     or setDutyCycleCommand() commands and record steady-state
 *     velocity/voltage/current from SmartDashboard.
 *   - With SysId: run sysIdQuasistatic() and sysIdDynamic() in both directions,
 *     then load the resulting WPILog into the SysId tool to fit kS/kV/kA.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class Flywheel extends SubsystemBase {

    private final TalonFX m_flywheelMotor;
    private final DutyCycleOut m_dutyCycleRequest;

    private final VoltageOut m_voltageRequest = new VoltageOut(0);

    // Built in the constructor: the lambdas below reference m_flywheelMotor, which
    // is not assigned until the constructor runs.
    private final SysIdRoutine m_sysIdRoutine;


    // The duty cycle (percent output, 0.0 to kFlywheelMaxDutyCycle) currently
    // commanded to the motor. Changed by the increase/decrease commands or
    // setDutyCycleCommand(), and applied continuously by runShooterComrmand().
    private double m_currentDutyCycle = kFlywheelDefaultDutyCycle;

    /**
     * Runs the quasistatic (slow voltage ramp) SysId test, used to identify kS and kV.
     *
     * @param direction which way to ramp the flywheel
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    /**
     * Runs the dynamic (voltage step) SysId test, used to identify kA.
     *
     * @param direction which way to step the flywheel
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    /** Creates a new Flywheel subsystem, configuring the shooter's TalonFX and its SysId routine. */
    public Flywheel() {
        m_flywheelMotor = new TalonFX(kFlywheelMotorID);
        m_dutyCycleRequest = new DutyCycleOut(0.0);

        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_flywheelMotor.getConfigurator().apply(flywheelConfig);

        m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(), // default ramp rate/step voltage/timeout — tune later if needed
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> m_flywheelMotor.setControl(m_voltageRequest.withOutput(volts.in(Volts))),
                log -> {
                    log.motor("flywheel")
                       .voltage(m_flywheelMotor.getMotorVoltage().getValue())
                       .angularPosition(m_flywheelMotor.getPosition().getValue())
                       .angularVelocity(m_flywheelMotor.getVelocity().getValue());
                },
                this
            )
        );
    }

    // ---------- Available Public Commands -----------
    /**
     * Runs the shooter open-loop at whatever duty cycle is currently stored
     * (set via increaseShooterSpeedCommand / decreaseShooterSpeedCommand /
     * setDutyCycleCommand). Bind this as a default command or a "hold to
     * run" button.
     */
    public Command runShooterCommand() {
        return run(() -> {
            m_flywheelMotor.setControl(m_dutyCycleRequest.withOutput(m_currentDutyCycle));
        })
        .finallyDo(() -> m_flywheelMotor.stopMotor())
        .withName("runShooter");
    }

    public Command stopShooterCommand() {
        return runOnce(m_flywheelMotor::stopMotor);
    }

    /** Bumps the stored duty cycle target up by one increment, clamped to the max. */
    public Command increaseShooterSpeedCommand() {
        return runOnce(() -> {
            m_currentDutyCycle = MathUtil.clamp(
                m_currentDutyCycle + kFlywheelDutyCycleIncrement, 0.0, kFlywheelMaxDutyCycle);
        });
    }

    /** Bumps the stored duty cycle target down by one increment, clamped to zero. */
    public Command decreaseShooterSpeedCommand() {
        return runOnce(() -> {
            m_currentDutyCycle = MathUtil.clamp(
                m_currentDutyCycle - kFlywheelDutyCycleIncrement, 0.0, kFlywheelMaxDutyCycle);
        });
    }

    /**
     * Directly sets the stored duty cycle target to a specific value, e.g.
     * for testing fixed setpoints like 0.25 / 0.5 / 0.75 during
     * characterization.
     *
     * @param dutyCycle desired duty cycle, clamped to [0.0, kFlywheelMaxDutyCycle]
     */
    public Command setDutyCycleCommand(double dutyCycle) {
        return runOnce(() -> {
            m_currentDutyCycle = MathUtil.clamp(dutyCycle, 0.0, kFlywheelMaxDutyCycle);
        });
    }


    // ---------- Logging Methods ----------
    /**
     * Get speed of flywheel motor in RPS (rotations per second).
     * Note this is currently just the motor speed, not actual flywheel speed,
     * as it does not account for gear ratio between motor and flywheel.
     * @return The speed of the motor in RPS (rotations per second)
     */
    @Logged
    public double getShooterSpeedRPS() {
        return m_flywheelMotor.getVelocity().getValueAsDouble();
    }

    @Logged
    public double getShooterSpeedRPM() {
        return m_flywheelMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    @Logged
    public double getCommandedDutyCycle() {
        return m_currentDutyCycle;
    }



    // ----------- Periodic Robot Routines ---------

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
        // 1. Extract the raw numeric values from Phoenix 6 StatusSignals
        double actualVelocityRPS = m_flywheelMotor.getVelocity().getValueAsDouble();
        double actualPositionRotations = m_flywheelMotor.getPosition().getValueAsDouble();
        double motorCurrentAmps = m_flywheelMotor.getStatorCurrent().getValueAsDouble();
        double appliedVoltage = m_flywheelMotor.getMotorVoltage().getValueAsDouble();

        // 2. Push fields to NetworkTables for SmartDashboard / AdvantageScope
        //    This is the data to watch/record while stepping through duty
        //    cycle values by hand: commanded duty cycle vs. resulting
        //    voltage/velocity/current.
        SmartDashboard.putNumber("Flywheel/Commanded Duty Cycle", m_currentDutyCycle);
        SmartDashboard.putNumber("Flywheel/Applied Voltage (V)", appliedVoltage);
        SmartDashboard.putNumber("Flywheel/Actual Velocity (RPS)", actualVelocityRPS);
        SmartDashboard.putNumber("Flywheel/Actual Velocity (RPM)", actualVelocityRPS * 60.0);
        SmartDashboard.putNumber("Flywheel/Position (Rotations)", actualPositionRotations);
        SmartDashboard.putNumber("Flywheel/Stator Current (A)", motorCurrentAmps);
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }

    
}
