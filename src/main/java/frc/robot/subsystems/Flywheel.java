// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


import static frc.robot.Constants.IOConstants.*;

/**
 * Flywheel subsystem with both open-loop (duty cycle) control for
 * characterization and closed-loop (feedforward + PID) velocity control.
 *
 * Two ways to characterize are available:
 *   - By hand: step through a few duty cycle values with the increase/decrease
 *     or setDutyCycleCommand() commands and record steady-state
 *     velocity/voltage/current from SmartDashboard.
 *   - With SysId: run sysIdQuasistatic() and sysIdDynamic() in both directions,
 *     then load the resulting WPILog into the SysId tool to fit kS/kV/kA.
 *
 * Once kS/kV/kA (feedforward) and kP/kI/kD (feedback) are filled in in
 * Constants.IOConstants from that SysId data, runVelocityCommand() drives the
 * flywheel to the RPM set via setTargetRPMCommand() — the gains are currently
 * placeholders (0.0), so this command does nothing useful until they're tuned.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class Flywheel extends SubsystemBase {

    private final TalonFX m_flywheelMotor;
    private final DutyCycleOut m_dutyCycleRequest;

    private final VoltageOut m_voltageRequest = new VoltageOut(0);

    // Built in the constructor: the lambdas below reference m_flywheelMotor, which
    // is not assigned until the constructor runs.
    private final SysIdRoutine m_sysIdRoutine;

    // Closed-loop velocity control: feedforward (kS/kV/kA) plus a PID controller
    // on velocity error. Gains come from Constants.IOConstants and are still
    // placeholders (0.0) until fit from SysId data — see the class javadoc.
    private final SimpleMotorFeedforward m_feedforward =
        new SimpleMotorFeedforward(kFlywheel_kS, kFlywheel_kV, kFlywheel_kA);
    private final PIDController m_velocityController =
        new PIDController(kFlywheel_kP, kFlywheel_kI, kFlywheel_kD);

    // Target velocity in rotations per second for runVelocityCommand(). Set via
    // setTargetRPMCommand()/setTargetRPSCommand().
    private double m_targetRPS = kFlywheelDefaultTargetRPM / 60.0;

    // Simulation-only physics model so SysId/PID can be exercised without real
    // hardware. See kFlywheelSimGearRatio/kFlywheelSimMOIKgMetersSquared javadoc
    // in Constants.IOConstants: this plant is a guess, not a measured flywheel.
    private final FlywheelSim m_flywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(1), kFlywheelSimMOIKgMetersSquared, kFlywheelSimGearRatio),
        DCMotor.getKrakenX60(1));

    // The duty cycle (percent output, 0.0 to kFlywheelMaxDutyCycle) currently
    // commanded to the motor. Changed by the increase/decrease commands or
    // setDutyCycleCommand(), and applied continuously by runShooterComrmand().
    private double m_currentDutyCycle = kFlywheelDefaultDutyCycle;

    // Reused across setTargetRPM() calls rather than allocated per-call — the
    // Phoenix docs call out building control requests once and mutating them
    // with .withVelocity()/.withSlot() instead of constructing new ones.
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);

    // Last RPM passed to setTargetRPM(), kept only so periodic() can log
    // target vs. measured velocity for the Slot0/VelocityVoltage path.
    private double m_lastSlot0TargetRPM = 0.0;

    // Last kP value written to the TalonFX's Slot0, so periodic() can tell
    // when the SmartDashboard-tunable kP has changed and needs re-applying.
    private double m_lastAppliedSlot0kP = kFlywheelSlot0_kP;

    // SmartDashboard keys for live-tunable Slot0 kP and target RPM, read back
    // and re-applied each periodic() so they can be adjusted without redeploying.
    private static final String kTunableSlot0kPKey = "Flywheel/Slot0 Tunable kP";
    private static final String kTunableSlot0TargetRPMKey = "Flywheel/Slot0 Tunable Target RPM";

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

        // Slot0 gains for the Phoenix-native VelocityVoltage closed loop (see
        // setTargetRPM()). Applied once here rather than per-call since the
        // configurator writes to the TalonFX over CAN. kP is re-applied later
        // in periodic() if it's changed on SmartDashboard (see applySlot0kP()).
        applySlot0kP(kFlywheelSlot0_kP);

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

    /**
     * Runs the shooter in closed loop at whatever target velocity is currently
     * stored (set via setTargetRPMCommand() / setTargetRPSCommand()), using
     * feedforward (kS/kV/kA) plus PID (kP/kI/kD) on velocity error. Bind this
     * as a default command or a "hold to run" button once the gains have been
     * tuned from SysId data.
     */
    public Command runVelocityCommand() {
        return run(() -> {
            double currentRPS = m_flywheelMotor.getVelocity().getValueAsDouble();
            double feedforwardVolts = m_feedforward.calculate(m_targetRPS);
            double feedbackVolts = m_velocityController.calculate(currentRPS, m_targetRPS);
            m_flywheelMotor.setControl(m_voltageRequest.withOutput(feedforwardVolts + feedbackVolts));
        })
        .finallyDo(() -> m_flywheelMotor.stopMotor())
        .withName("runShooterVelocity");
    }

    /**
     * Directly sets the target velocity for runVelocityCommand(), e.g. for
     * testing fixed setpoints during PID tuning.
     *
     * @param targetRPM desired velocity in rotations per minute, clamped to [0, kFlywheelMaxRPM]
     */
    public Command setTargetRPMCommand(double targetRPM) {
        return runOnce(() -> {
            m_targetRPS = MathUtil.clamp(targetRPM, 0.0, kFlywheelMaxRPM) / 60.0;
        });
    }

    /**
     * Commands the flywheel directly to a target velocity using the TalonFX's
     * onboard Slot0 controller (kS/kV/kP/kI/kD applied in the constructor),
     * rather than the WPILib PIDController/SimpleMotorFeedforward path above.
     * Call this repeatedly (e.g. from a command's execute()) to hold a speed.
     *
     * @param rpm desired velocity in rotations per minute; the TalonFX takes
     *            rotations per second, hence the divide by 60
     */
    public void setTargetRPM(double rpm) {
        m_lastSlot0TargetRPM = rpm;
        m_flywheelMotor.setControl(m_velocityRequest.withVelocity(rpm / 60.0));
    }

    /**
     * Rebuilds Slot0Configs with the given kP (kS/kV/kI/kD unchanged from
     * Constants) and re-applies it to the TalonFX. Called once from the
     * constructor with the Constants default, and again from periodic()
     * whenever the SmartDashboard-tunable kP value changes, so kP can be
     * tuned live without redeploying code.
     *
     * @param kP proportional gain to write to Slot0
     */
    private void applySlot0kP(double kP) {
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kS = kFlywheelSlot0_kS;
        slot0Configs.kV = kFlywheelSlot0_kV;
        slot0Configs.kP = kP;
        slot0Configs.kI = kFlywheelSlot0_kI;
        slot0Configs.kD = kFlywheelSlot0_kD;
        StatusCode slot0Status = m_flywheelMotor.getConfigurator().apply(slot0Configs);
        if (!slot0Status.isOK()) {
            System.out.println("Flywheel Slot0Configs apply failed: " + slot0Status.toString());
        }
        m_lastAppliedSlot0kP = kP;
    }

    /** Commands zero output, stopping the flywheel. */
    public void stop() {
        m_flywheelMotor.stopMotor();
    }

    /**
     * @return current flywheel velocity in rotations per minute, read from the
     *         TalonFX (motor-shaft speed; does not account for any gear ratio)
     */
    public double getVelocityRPM() {
        return m_flywheelMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    /**
     * Command factory: drives the flywheel to a fixed target velocity using
     * the Slot0/VelocityVoltage path while it runs, and stops the flywheel
     * when the command ends (interrupted or otherwise) so a button release
     * doesn't leave it spinning.
     *
     * @param rpm target velocity in rotations per minute
     */
    public Command runAtRPM(double rpm) {
        return run(() -> setTargetRPM(rpm))
            .finallyDo(() -> stop())
            .withName("runAtRPM");
    }

    /**
     * Command factory: like runAtRPM(), but reads its target velocity from
     * the SmartDashboard-tunable "Flywheel/Slot0 Tunable Target RPM" entry
     * each cycle instead of a fixed value, for live tuning on the bench.
     */
    public Command runAtTunableRPM() {
        return run(() -> setTargetRPM(
                SmartDashboard.getNumber(kTunableSlot0TargetRPMKey, kFlywheelDefaultTargetRPM)))
            .finallyDo(() -> stop())
            .withName("runAtTunableRPM");
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
        SmartDashboard.putNumber("Flywheel/Target Velocity (RPM)", m_targetRPS * 60.0);
        SmartDashboard.putNumber("Flywheel/Slot0 Target Velocity (RPM)", m_lastSlot0TargetRPM);
        SmartDashboard.putNumber("Flywheel/Slot0 Measured Velocity (RPM)", getVelocityRPM());

        // 3. Live-tunable Slot0 kP and target RPM: publish defaults the first
        //    time (getNumber falls back to the given default if the key isn't
        //    present yet), then re-apply kP to the TalonFX if it was edited.
        SmartDashboard.putNumber(kTunableSlot0TargetRPMKey,
            SmartDashboard.getNumber(kTunableSlot0TargetRPMKey, kFlywheelDefaultTargetRPM));
        double tunableKP = SmartDashboard.getNumber(kTunableSlot0kPKey, kFlywheelSlot0_kP);
        SmartDashboard.putNumber(kTunableSlot0kPKey, tunableKP);
        if (tunableKP != m_lastAppliedSlot0kP) {
            applySlot0kP(tunableKP);
        }
        SmartDashboard.putNumber("Flywheel/Applied Voltage (V)", appliedVoltage);
        SmartDashboard.putNumber("Flywheel/Actual Velocity (RPS)", actualVelocityRPS);
        SmartDashboard.putNumber("Flywheel/Actual Velocity (RPM)", actualVelocityRPS * 60.0);
        SmartDashboard.putNumber("Flywheel/Position (Rotations)", actualPositionRotations);
        SmartDashboard.putNumber("Flywheel/Stator Current (A)", motorCurrentAmps);
    }

    @Override
    public void simulationPeriodic() {
        // Drive the physics model from whatever voltage the TalonFX's sim state
        // says was commanded (works for both DutyCycleOut and VoltageOut, since
        // Phoenix converts duty cycle to an equivalent voltage internally), then
        // feed the resulting velocity/position back into the TalonFX sim state
        // so getVelocity()/getPosition() reflect it next loop.
        TalonFXSimState simState = m_flywheelMotor.getSimState();
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        m_flywheelSim.setInputVoltage(simState.getMotorVoltage());
        m_flywheelSim.update(0.020);

        double simRPS = m_flywheelSim.getAngularVelocityRPM() / 60.0;
        simState.setRotorVelocity(simRPS);
        simState.addRotorPosition(simRPS * 0.020);
    }

    
}
