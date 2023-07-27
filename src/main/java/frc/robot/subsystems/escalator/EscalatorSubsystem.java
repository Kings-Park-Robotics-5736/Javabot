package frc.robot.subsystems.escalator;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Types.FeedForwardConstants;
import frc.robot.Types.PidConstants;


public class EscalatorSubsystem extends SubsystemBase {

    private CANSparkMax m_motor;
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;
    private ElevatorFeedforward m_feedforward;
    private final String m_name;

    private TrapezoidProfile profile;
    private double startTime = 0;
    private int staleCounter = 0;
    private double lastPosition = 0;

    private final double kStaleTolerance = .5;
    private final double kDiffThreshold = 0.15;
    private final int kStaleThreshold = 10;

    public EscalatorSubsystem(PidConstants pidValues, FeedForwardConstants ffValues, byte deviceId, String _name) {

        m_motor = new CANSparkMax(deviceId, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(true);

        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();

        m_name = _name;

        m_pidController.setP(pidValues.p);
        m_pidController.setI(pidValues.i);
        m_pidController.setD(pidValues.d);
        m_pidController.setIZone(0);
        m_pidController.setFF(0);
        m_pidController.setOutputRange(-1, 1);
        m_feedforward = new ElevatorFeedforward(ffValues.ks, ffValues.kg, ffValues.kv, ffValues.ka);

    }

    @Override
    public void periodic() {

    }

    public void stopEscalator() {
        setSpeed(0);
    }

    /**
     * @brief Runs the escalator at a given speed (-1 to 1) in manual mode until interrupted
     * @param getSpeed a lambda that takes no arguments and returns the desired speed of the escalator [ () => double ]
     * @return the composed command to manually drive the escalator
     */
    public Command RunEscalatorManualSpeedCommand(DoubleSupplier getSpeed) {
        return new FunctionalCommand(
                () -> {
                },
                () -> setSpeed(getSpeed.getAsDouble()),
                (interrupted) -> stopEscalator(),
                () -> false, this);
    }

    /**
     * @brief Runs the escalator to a given position in absolute rotations
     * @param position absolute position to run to (not a lambda)
     * @return the composed command to run the escalator to a given positions
     */
    public Command RunEscalatorToPositionCommand(double position) {
        return new FunctionalCommand(
                () -> InitMotionProfile(position),
                () -> RunEscalator(),
                (interrupted) -> stopEscalator(),
                () -> isFinished(), this);
    }

    /**
     * 
     * Set the speed of the intake motor (-1 to 1)
     * 
     * @param speed
     */
    private void setSpeed(double speed) {
        m_motor.set(speed);
        SmartDashboard.putNumber("Escalator Position" + m_name, m_encoder.getPosition());
    }

    /**
     * @brief Initializes the motion profile for the elevator
     * @param setpoint of the motor, in absolute rotations
     */
    private void InitMotionProfile(double setpoint) {
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(60, 150),
                new TrapezoidProfile.State(setpoint, 0),
                new TrapezoidProfile.State(m_encoder.getPosition(), 0));

        startTime = Timer.getFPGATimestamp();

    }

    /**
     * @brief Checks if the elevator has reached its target
     * @return true if the elevator has reached its target, false otherwise
     */
    private Boolean escalatorReachedTarget() {

        // check if the elevator has stalled and is no longer moving
        // if it hasn't moved (defined by encoder change less than kDiffThreshold),
        // increment the stale counter
        // if it has moved, reset the stale counter
        if (Math.abs(m_encoder.getPosition() - lastPosition) < kDiffThreshold) {
            staleCounter++;
        } else {
            staleCounter = 0;
        }
        lastPosition = m_encoder.getPosition();

        // calculate the difference between the current position and the motion profile
        // final position
        double delta = Math.abs(m_encoder.getPosition() - profile.calculate(profile.totalTime()).position);

        // we say that the elevator has reached its target if it is within
        // kDiffThreshold of the target,
        // or if it has been within a looser kStaleTolerance for kStaleThreshold cycles
        return delta < kDiffThreshold || (delta < kStaleTolerance && staleCounter > kStaleThreshold);
    }

    private Boolean isFinished() {
        var isFinished = profile.isFinished(Timer.getFPGATimestamp() - startTime) && escalatorReachedTarget();
        SmartDashboard.putBoolean("isfinished " + m_name, isFinished);
        return isFinished;
    }

    /**
     * @brief Runs the escalator to a given position in absolute rotations
     * 
     */
    private void RunEscalator() {

        double currTime = Timer.getFPGATimestamp();
        var setpoint = profile.calculate(currTime - startTime);
        double ff = m_feedforward.calculate(setpoint.velocity);

        m_pidController.setReference(setpoint.position, CANSparkMax.ControlType.kPosition, 0, ff,
                ArbFFUnits.kVoltage);

        SmartDashboard.putNumber("ProcessVariable" + m_name, m_encoder.getPosition());
        SmartDashboard.putNumber("FF" + m_name, ff);
        SmartDashboard.putNumber("setpoint_pos" + m_name, setpoint.position);
        SmartDashboard.putNumber("time " + m_name, currTime - startTime);
    }

}