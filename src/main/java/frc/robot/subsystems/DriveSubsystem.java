// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.RobotConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class DriveSubsystem extends SubsystemBase {
    // --- motors ---
    private final SparkMax m_leftLeader;
    private final SparkMax m_leftFollower;
    private final SparkMax m_rightLeader;
    private final SparkMax m_rightFollower;

    // --- Differential Drive subsystem 
    private final DifferentialDrive m_differentialDrive;

    // --- Encoders ---
    private final CANcoder m_rightEncoder;

    // --- Gryo ----
    private final Pigeon2 m_pigeon2 = new Pigeon2(kPigeon2ID);  // need to install on CAN bus
    

    // --- Odometry ---
    private final DifferentialDriveOdometry m_odometry;
     // No physical left encoder exists. This is a running total built one
    // signed delta at a time, so it stays continuous across mode switches.
    private double m_virtualLeftDistanceMeters = 0.0;
    private double m_lastRightDistanceMeters;
    private Rotation2d m_lastHeading; // heading on previous scheduler tick
    private Rotation2d m_turnTargetHeading;  // helpers for turning command and keeping track of state 
    private double m_angularVelocityRadPerSecEst; // estimate of angular velocity baed on heading change

    private final Field2d m_field = new Field2d(); // field object for display

    /** Creates a new DriveSubsystem, configuring the KitBot's differential drivetrain motors and right-side CANcoder. */
    public DriveSubsystem() {

        // Create brushed motors for a KitBot-style CIM drivetrain
        m_leftLeader = new SparkMax(kLeftLeaderId, MotorType.kBrushed);
        m_leftFollower = new SparkMax(kLeftFollowerId, MotorType.kBrushed);
        m_rightLeader = new SparkMax(kRightLeaderId, MotorType.kBrushed);
        m_rightFollower = new SparkMax(kRightFollowerId, MotorType.kBrushed);


        // Left leader: invert so that positive values drive both sides forward
        SparkMaxConfig m_leftLeaderConfig = new SparkMaxConfig();
        m_leftLeaderConfig.voltageCompensation(12);
        m_leftLeaderConfig.smartCurrentLimit(kDriveMotorCurrentLimit);
        m_leftLeaderConfig.inverted(kLeftLeaderReversed);
        m_leftLeader.configure(m_leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Right leader: not inverted
        SparkMaxConfig m_rightLeaderConfig = new SparkMaxConfig();
        m_rightLeaderConfig.voltageCompensation(12);
        m_rightLeaderConfig.smartCurrentLimit(kDriveMotorCurrentLimit);
        m_rightLeaderConfig.inverted(kRightLeaderReversed);
        m_rightLeader.configure(m_rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Followers mirror their respective leaders
        SparkMaxConfig m_leftFollowerConfig = new SparkMaxConfig();
        m_leftFollowerConfig.follow(m_leftLeader);
        m_leftFollower.configure(m_leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig m_rightFollowerConfig = new SparkMaxConfig();
        m_rightFollowerConfig.follow(m_rightLeader);
        m_rightFollower.configure(m_rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Built-in encoders
        m_rightEncoder = new CANcoder(kRightEncoderID);
        

        // Odometry initializatoin: reset encoders and gyro and initilaize odometry object.
        resetEncoders();
        m_pigeon2.reset();
        m_lastHeading = m_pigeon2.getRotation2d();
    
        // initialize odometry with desired starting location (0,0) is bottom-left corner of field.
        m_odometry = new DifferentialDriveOdometry(
            m_pigeon2.getRotation2d(),
            m_virtualLeftDistanceMeters, getRightDistanceMeters(),
            new Pose2d(1.616, 4.035, Rotation2d.kZero)); // 4th arg

        // initialize differential drive object
        m_differentialDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);

        SmartDashboard.putData("Field", m_field);
    }

    // ----------- Driving Commands --------------

    /**
     * A split-stick arcade command, with forward/backward controlled by the left hand, 
     * and turning controlled by right; values should be negated when sent from controller
     * due to differences in coordinate orientation. 
     * @param fwd 
     * @param rot
     * @return
     */
    public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
        return run(() -> m_differentialDrive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble()))
            .withName("arcadeDrive");
    }

    public Command stopDriveCommand() {
        return runOnce( () -> this.stopMotors() );
    }

    public Command resetEncoderCommand() {
        return runOnce( () -> this.resetEncoders() );
    }

     /** 
     * Stops drivetrain motors immediately
     */
    public void stopMotors() {
        m_differentialDrive.stopMotor();
    }


    // --------- Odometry and Helper Functions -----------

    /**
     * Returns the distance traveled by the right side in meters.
     */
    @Logged
    public double getRightDistanceMeters() {
        return m_rightEncoder.getPosition().getValueAsDouble() * kDistancePerRotationMeters;
    }

    /**
     * Returns the distance traveled by the left side in meters.
     * Note: this is based on the virtual (i.e., fake) left encoder.
     */
    @Logged
    public double getLeftDistanceMeters() {
        return m_virtualLeftDistanceMeters;
    }

    /**
     * Returns the robot heading as a Rotation2d
     * @return
     */
    public Rotation2d getHeadingRotation2d() {
        return m_pigeon2.getRotation2d();
    }

    /**
     * Returns the robot heading as a Rotation2d
     * @return
     */
    @Logged
    public double getHeadingDegrees() {
        return m_pigeon2.getRotation2d().getDegrees();
    }

    /**
     * Returns the robot pose, according to odometry. 
     * @return
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }


    /**
     * Returns the average distance traveled by both sides in meters.
     */
    @Logged
    public double getAverageDistanceMeters() {
        return  getLeftDistanceMeters() + getRightDistanceMeters() / 2.0;
    }

    /** Returns current velocity estimate in m/s based on the CANcoder 
     * that is mounted directly to the output axel. 
     */
    @Logged()
    public double getVelocityMetersPerSecond() {
        return m_rightEncoder.getVelocity().getValueAsDouble() * kDistancePerRotationMeters
            - m_angularVelocityRadPerSecEst * kDriveTrackWidthMeters / 2.0 ; 
    }

    @Logged 
    public double getRightVelocityMetersPerSecond() {
        return m_rightEncoder.getVelocity().getValueAsDouble() * kDistancePerRotationMeters;
    }

    @Logged
    public double getLeftVelocityMetersPerSecond() {
        return getRightVelocityMetersPerSecond() - m_angularVelocityRadPerSecEst * kDriveTrackWidthMeters;
    }
    /** Returns the left velocity estimate. Note: this is an estimated 
     * reading based on the gyro and right side, rather than from an encoder. 
     */

    /** Resets both drive encoders to zero. */
    public void resetEncoders() {

        m_rightEncoder.setPosition(0);
        m_virtualLeftDistanceMeters = 0.0;
    }


    // ------------ Feedforward Control and Testing ----------------- 
     /* Helper methods for setting drive commands in meters per second */
    private final SimpleMotorFeedforward m_feedforward = 
        new SimpleMotorFeedforward(kDriveBase_kS, kDriveBase_kV, kDriveBase_kA); // Replace with your actual constants

    /**
     * Commands the robot to drive at a specific target velocity using only feedforward.
     * 
     * @param targetVelocityMetersPerSecond The desired speed in m/s
     */
    public Command testFeedforwardCommand(double targetVelocityMetersPerSecond) {
        return run(() -> {
            // Calculate the required voltage to achieve the target velocity
            double appliedVoltage = m_feedforward.calculate(targetVelocityMetersPerSecond);

            // Bypass DifferentialDrive and apply the voltage directly to the motor controllers
            m_leftLeader.setVoltage(appliedVoltage);
            m_rightLeader.setVoltage(appliedVoltage);

            // Feed the Motor Safety watchdog so it doesn't disable our motors
            m_differentialDrive.feed();
        })
        .finallyDo(() -> this.stopMotors())
        .withName("testFeedforward");
    }


    
    // ------- Additional Helper Functions
    // helper function: convert speed in meters per second to RPM
    private static double mpsToMotorRpm(double mps) {
        return mps / kDistancePerRotationMeters * kDriveGearRatio * 60.0;
    }

    private void setSpeeds(double leftVelocityMetersPerSecond, double rightVelocityMetersPerSecond) {
        double leftVoltage = m_feedforward.calculate(leftVelocityMetersPerSecond);
        double rightVoltage = m_feedforward.calculate(rightVelocityMetersPerSecond);

        m_leftLeader.setVoltage(leftVoltage);
        m_rightLeader.setVoltage(rightVoltage);

        m_differentialDrive.feed();
    }


    private void setSpeeds(DifferentialDriveWheelSpeeds differentialDriveWheelSpeeds) {

        double leftVelocityMetersPerSecond = differentialDriveWheelSpeeds.leftMetersPerSecond;
        double rightVelocityMetersPerSecond = differentialDriveWheelSpeeds.rightMetersPerSecond;

        double leftVoltage = m_feedforward.calculate(leftVelocityMetersPerSecond);
        double rightVoltage = m_feedforward.calculate(rightVelocityMetersPerSecond);

        m_leftLeader.setVoltage(leftVoltage);
        m_rightLeader.setVoltage(rightVoltage);
    }

   

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
        // update odometry

        // get existing encoder readings: 
        final Rotation2d currentHeading = m_pigeon2.getRotation2d();
        final double currentRight = getRightDistanceMeters();

        // compute deltaRight and delta_headings (for purpose of faking deltaLeft)
        final double deltaRight = currentRight - m_lastRightDistanceMeters;
        final double deltaHeadingRadians = currentHeading.minus(m_lastHeading).getRadians();

        // estimate angular velocity
        m_angularVelocityRadPerSecEst = deltaHeadingRadians / kRobotLoopPeriod;

        // update virtual left distance; note the +=g
        m_virtualLeftDistanceMeters += deltaRight - kDriveTrackWidthMeters * deltaHeadingRadians;

        m_lastRightDistanceMeters = currentRight;
        m_lastHeading = currentHeading;

        m_odometry.update(currentHeading, m_virtualLeftDistanceMeters, currentRight);

        SmartDashboard.putNumber("Odometry/X", m_odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry/Y", m_odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Odometry/HeadingDeg", m_odometry.getPoseMeters().getRotation().getDegrees());

        m_field.setRobotPose(m_odometry.getPoseMeters()); // estimate
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }


   
}
