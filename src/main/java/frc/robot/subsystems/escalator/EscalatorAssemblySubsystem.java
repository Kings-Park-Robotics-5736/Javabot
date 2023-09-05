package frc.robot.subsystems.escalator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Types.DirectionType;

//1 is escalator
//2 is lower intake
//8 is flipper 
//9 is elevator

public class EscalatorAssemblySubsystem extends SubsystemBase {

    private EscalatorSubsystem escalator;
    private ElevatorSubsystem elevator;
    private FlipperSubsystem flipper;

    public EscalatorAssemblySubsystem() {
        escalator = new EscalatorSubsystem(Constants.EscalatorConstants.pidValues,
                Constants.ElevatorContstraints.limits,
                Constants.EscalatorConstants.ffValues, Constants.EscalatorConstants.deviceId, "Escalator");

        elevator = new ElevatorSubsystem(Constants.ElevatorContstraints.pidValues,
                Constants.ElevatorContstraints.limits,
                Constants.ElevatorContstraints.deviceId, "Elevator");

        flipper = new FlipperSubsystem(Constants.FlipperContstraints.deviceId, Constants.FlipperContstraints.speed,
                "Flipper");
    }

    @Override
    public void periodic() {

    }

    /*****************************************
     * Manual Control
     ******************************************/

    public Command RunEscalatorManualSpeedCommand(DoubleSupplier getSpeed) {
        return escalator.RunEscalatorManualSpeedCommand(getSpeed).unless(() -> !elevator.IsElevatorUp());
    }

    public Command RunElevatorManualSpeedCommand(DoubleSupplier getSpeed, Boolean ignoreLimits) {
        return elevator.RunElevatorManualSpeedCommand(getSpeed, ignoreLimits)
                .unless(() -> !escalator.IsEscalatorDown());
    }

    public Command RunFlipperManualSpeedCommand(DoubleSupplier getSpeed) {
        return flipper.RunFlipperManualSpeedCommand(getSpeed);
    }

    /*****************************************
     * Auto Control
     ******************************************/
    public Command RunFlipperToPositionCommand(DirectionType d) {
        return flipper.RunFlipperToPositionCommand(d);
    }

    public Command RunFlipperUpCommand() {
        return flipper.RunFlipperToPositionCommand(DirectionType.UP);
    }

    public Command RunFlipperDownCommand() {
        return flipper.RunFlipperToPositionCommand(DirectionType.DOWN);
    }


    public Command RunEscalatorToPositionCommand(double position) {
        return escalator.RunEscalatorToPositionCommand(position).unless(() -> !elevator.IsElevatorUp());
    }

    public Command RunEscalatorToHighScore() {
        return escalator.RunEscalatorToPositionCommand(61).unless(() -> !elevator.IsElevatorUp());
    }

    public Command RunEscalatorToMidScore() {
        return escalator.RunEscalatorToPositionCommand(38.8).unless(() -> !elevator.IsElevatorUp());
    }

    public Command RunEscalatorToBottom() {
        return escalator.RunEscalatorToPositionCommand(0).unless(() -> !elevator.IsElevatorUp());
    }

    
    public Command RunElevatorToPositionCommand(double position) {
        return elevator.RunElevatorToPositionCommand(position).unless(() -> !escalator.IsEscalatorDown());
    }

    public Command RunElevatorUpCommand() {
        return elevator.RunElevatorToPositionCommand(0).unless(() -> !escalator.IsEscalatorDown());
    }

    public Command RunElevatorDownCommand() {
        return elevator.RunElevatorToPositionCommand(59).unless(() -> !escalator.IsEscalatorDown());
    }

    /******************************************
     * Composed Commands
     *****************************************/

    public Command ScoreHigh() {
        // NOTE - parenthesis are very important here. The below is valid,
        return RunElevatorUpCommand().andThen(RunEscalatorToHighScore()).andThen(RunFlipperUpCommand())
                .andThen((RunFlipperDownCommand()).alongWith(RunEscalatorToBottom()));

        // but this is not
        // return
        // RunElevatorUpCommand().andThen(RunEscalatorToHighScore()).andThen(RunFlipperUpCommand()).andThen(RunFlipperDownCommand()).alongWith(RunEscalatorToBottom());
        // that's because we want to say "run the flipper down and the escalator to
        // bottom at the same time" AFTER we run the elevator, escalator, and flipper
        // up. Without the parenthesis, the "alongWith" is applied to everything before
        // it, which is not what we want and causes an exception
    }

    public Command ScoreMid() {
        return RunElevatorUpCommand().andThen(RunEscalatorToMidScore()).andThen(RunFlipperUpCommand())
                .andThen((RunFlipperDownCommand()).alongWith(RunEscalatorToBottom()));
    }

    /*****************************************
     * Misc Control
     ******************************************/
    public Command ResetElevatorEncoderCommand() {
        return elevator.ResetElevatorEncoderCommand();
    }

    public Command ResetEscalatorEncoderCommand() {
        return escalator.ResetEscalatorEncoderCommand();
    }

}
