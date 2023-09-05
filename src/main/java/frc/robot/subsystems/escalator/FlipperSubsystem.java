package frc.robot.subsystems.escalator;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Types.DirectionType;

public class FlipperSubsystem extends SubsystemBase {

    private WPI_TalonSRX m_motor;
    private final String m_name;

    private final double m_defaultSpeed;

    public FlipperSubsystem(byte deviceId, double speed, String _name) {

        m_motor = new WPI_TalonSRX(deviceId);
        m_motor.configFactoryDefault();

        m_defaultSpeed = speed;

        m_name = _name;

    }

    @Override
    public void periodic() {

    }

    /**
     * @brief Runs the escalator at a given speed (-1 to 1) in manual mode until
     *        interrupted
     * @param getSpeed a lambda that takes no arguments and returns the desired
     *                 speed of the escalator [ () => double ]
     * @return the composed command to manually drive the escalator
     */
    public Command RunFlipperManualSpeedCommand(DoubleSupplier getSpeed) {
        return new FunctionalCommand(
                () -> {
                },
                () -> setSpeed(getSpeed.getAsDouble()),
                (interrupted) -> stopFlipper(),
                () -> false, this);
    }

    public Command RunFlipperToPositionCommand(DirectionType direction) {
        final double speed;
        final double time;
        if (direction == DirectionType.DOWN) {
            speed = -m_defaultSpeed;
            time = .6;
        } else {
            speed = m_defaultSpeed;
            time = .8;
        }
        return Commands.runOnce(() -> setSpeed(speed)).andThen(Commands.waitSeconds(time))
                .andThen(runOnce(() -> setSpeed(0)));
    }

    public void stopFlipper() {
        setSpeed(0);
    }

    /**
     * 
     * Set the speed of the intake motor (-1 to 1)
     * 
     * @param speed
     */
    private void setSpeed(double speed) {
        m_motor.set(speed);

    }

}