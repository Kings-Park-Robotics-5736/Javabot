package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;


/**
 * Limelight camera class
 */
public class Limelight {
    private static NetworkTable table;
    private static NetworkTableEntry tx;
    private static NetworkTableEntry ty;
    private static NetworkTableEntry tv;
    private static NetworkTableEntry ta;
    private static NetworkTableEntry camMode;
    private static NetworkTableEntry ledMode;

    public enum LEDMode {
        PIPELINE(0),
        OFF(1),
        BLINK(2),
        ON(3);

        private int value;

        private LEDMode(int mode) {
            this.value = mode;
        }
    }

    public enum CamMode {
        VISION(0),
        DRIVER(1);

        private int value;

        private CamMode(int mode) {
            this.value = mode;
        }
    }

    /**
     * Camera sends data to network table, get table and values when creating instance of Limelight
     */
    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx"); // horizontal offset (-29.8 - 29.8 degrees)
        ty = table.getEntry("ty"); // vertical offset (-24.85 - 24.85 degrees)
        tv = table.getEntry("tv"); // valid target (0 - 1)
        ta = table.getEntry("ta"); // target area (0% - 100% of image)
        ledMode = table.getEntry("ledMode"); // LED state (0-3)
        camMode = table.getEntry("camMode"); // operation mode (0-1)
    }

    /**
     * Get current LED mode
     * @return int (0-4)
     */
    public int getLEDMode() {
        return ledMode.getNumber(0).intValue();
    }

    /**
     * Set LED mode
     * @param mode LEDMode(0-4)
     */
    public void setLEDMode(LEDMode mode) {
        ledMode.setNumber(mode.value);
    }

    /**
     * Set LED mode to ON
     */
    public void setLEDOn() {
        ledMode.setNumber(LEDMode.ON.value);
    }

    /**
     * Set LED mode to OFF
     */
    public void setLEDOff() {
        ledMode.setNumber(LEDMode.OFF.value);
    }

    /**
     * Set LED mode to BLINK
     */
    public void setLEDBlink() {
        ledMode.setNumber(LEDMode.BLINK.value);
    }

    /**
     * Get current camera mode
     * @return int (0-1)
     */
    public int getCamMode() {
        return camMode.getNumber(0).intValue();
    }

    /**
     * Set camera mode
     * @param mode CamMode(0-1)
     */
    public void setCamMode(CamMode mode) {
        camMode.setNumber(mode.value);
    }

    /**
     * Preset operation mode - LED set to OFF, camera set to DRIVER
     */
    public void setModeDriver() {
        this.setLEDMode(LEDMode.OFF);
        this.setCamMode(CamMode.DRIVER);
    }

    /**
     * Preset operation mode - LED set to ON, camera set to VISION
     */
    public void setModeVision() {
        this.setLEDMode(LEDMode.ON);
        this.setCamMode(CamMode.VISION);
    }

    /**
     * Get horizontal offset to target
     * @return -29.8 - 29.8 degrees
     */
    public double getTargetOffsetX() {
        return tx.getDouble(0.0);
    }

    /**
     * Get vertical offset to target
     * @return -24.85 - 24.85 degrees
     */
    public double getTargetOffsetY() {
        return ty.getDouble(0.0);
    }

    /**
     * Check for a detected target
     * @return boolean - true if target is found else false
     */
    public boolean checkValidTarget() {
        if (tv.getNumber(0).intValue() == 1) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Get target area
     * @return 0% - 100% of  image
     */
    public double getTargetArea() {
        return ta.getDouble(0.0);
    }

    /**
     * Command - toggle LED ON/OFF
     * @return Command
     *
    public Command ToggleLEDCommand() {
        return new FunctionalCommand(
            () -> {},
            () -> this.setLEDOn(),
            (interrupted) -> this.setLEDOff(),
            () -> false, this);
    }
    */

    /**
     * Command - toggle limelight LED ON/OFF
     */
    public void toggleLED() {
        if (this.getLEDMode() == LEDMode.ON.value) {
            this.setLEDOff();
        } else {
            this.setLEDOn();
        }
    }

}
