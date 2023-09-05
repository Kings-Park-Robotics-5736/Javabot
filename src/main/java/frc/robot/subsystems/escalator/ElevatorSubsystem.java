package frc.robot.subsystems.escalator;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Types.Limits;
import frc.robot.utils.Types.PidConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private WPI_TalonSRX m_motor;
    private DutyCycleEncoder m_encoder;
    private final String m_name;

    private final Limits m_limits;

    private int staleCounter = 0;
    private double lastPosition = 0;

    private final double kStaleTolerance = .5;
    private final double kDiffThreshold = 0.15;
    private final int kStaleThreshold = 10;

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(100, 100);
    private final ProfiledPIDController m_controller = new ProfiledPIDController(0.5, 0.0, 0.007, m_constraints,
            Constants.kDt);

    public ElevatorSubsystem(PidConstants pidValues, Limits limits, byte deviceId, String _name) {

        m_motor = new WPI_TalonSRX(deviceId);
        m_motor.configFactoryDefault();
        m_encoder = new DutyCycleEncoder(0);

        m_motor.setInverted(true);

        m_name = _name;
        m_limits = limits;

    }

    @Override
    public void periodic() {

    }

    public Boolean IsElevatorUp() {
        return m_encoder.get() < m_limits.low + 10;
    }

    public Boolean IsElevatorDown() {
        return m_encoder.get() > m_limits.high - 10;
    }

    /**
     * @brief Runs the escalator at a given speed (-1 to 1) in manual mode until
     *        interrupted
     * @param getSpeed a lambda that takes no arguments and returns the desired
     *                 speed of the escalator [ () => double ]
     * @return the composed command to manually drive the escalator
     */
    public Command RunElevatorManualSpeedCommand(DoubleSupplier getSpeed, Boolean ignoreLimits) {
        return new FunctionalCommand(
                () -> {
                },
                () -> setSpeed(getSpeed.getAsDouble()),
                (interrupted) -> stopElevator(),
                () -> !isWithinLimits() && !ignoreLimits, this);
    }

    public Command RunElevatorToPositionCommand(double position) {
        return new FunctionalCommand(
                () -> InitMotionProfile(position),
                () -> RunElevator(position),
                (interrupted) -> stopElevator(),
                () -> isFinished(), this);
    }

    public Command ResetElevatorEncoderCommand() {
        return this.runOnce(() -> resetEncoder());
    }

    public void stopElevator() {
        setSpeed(0);
    }

    private void resetEncoder() {
        m_encoder.reset();
    }

    private void InitMotionProfile(double setpoint) {
        m_controller.reset(m_encoder.getDistance());
        m_controller.setTolerance(1.0);
        m_controller.setGoal(new TrapezoidProfile.State(setpoint, 0));

    }

    private void RunElevator(double goal) {
        double output = m_controller.calculate(m_encoder.getDistance(), goal);
        setSpeed(output);
    }

    /**
     * @brief Checks if the elevator has reached its target
     * @return true if the elevator has reached its target, false otherwise
     */
    private Boolean elevatorReachedTarget() {

        // check if the elevator has stalled and is no longer moving
        // if it hasn't moved (defined by encoder change less than kDiffThreshold),
        // increment the stale counter
        // if it has moved, reset the stale counter
        if (Math.abs(m_encoder.get() - lastPosition) < kDiffThreshold) {
            staleCounter++;
        } else {
            staleCounter = 0;
        }
        lastPosition = m_encoder.get();

        // calculate the difference between the current position and the motion profile
        // final position
        double delta = Math.abs(m_encoder.get() - m_controller.getGoal().position);

        // we say that the elevator has reached its target if it is within
        // kDiffThreshold of the target,
        // or if it has been within a looser kStaleTolerance for kStaleThreshold cycles
        return delta < kDiffThreshold || (delta < kStaleTolerance && staleCounter > kStaleThreshold);
    }

    private Boolean isFinished() {
        var isFinished = (m_controller.atGoal() && elevatorReachedTarget()) || !isWithinLimits();
        SmartDashboard.putBoolean("isfinished " + m_name, isFinished);
        return isFinished;
    }

    // on this motor, negative speed is up with more negative (lower) encoder counts
    private Boolean isWithinLimits() {
        return (m_motor.get() < 0 && m_encoder.get() > m_limits.low)
                || (m_motor.get() > 0 && m_encoder.get() < m_limits.high);
    }

    /**
     * 
     * Set the speed of the intake motor (-1 to 1)
     * 
     * @param speed
     */
    private void setSpeed(double speed) {
        m_motor.set(speed);
        SmartDashboard.putNumber("Elevator speed" + m_name, speed);
        SmartDashboard.putNumber("Elevator Distance" + m_name, m_encoder.getDistance());
        SmartDashboard.putNumber("Elevator setpoint" + m_name, m_controller.getSetpoint().position);
    }

}