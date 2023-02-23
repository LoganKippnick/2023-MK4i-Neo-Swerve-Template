package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Rumble {

    private final Joystick controller;
    private final RumbleType rumbleType;
    private final double power;
    private final Timer buzzTimer;

    private boolean isRumbling;
    /**
     * Returns whether the controller is rumbling.
     * @return True if the controller is rumbling.
     */
    public boolean isRumbling() {
        return isRumbling;
    }

    private boolean isEnabled;
    /**
     * Returns if the Rumble is enabled.
     * @return True if the Rumble is enabled.
     */
    public boolean isEnabled() {
        return isEnabled;
    }
    /**
     * Sets whether the Rumble is enabled. If false, stops the rumbling and buzzing instantly.
     * @param isEnabled True if the Rumble should be enabled, false if it should be disabled.
     */
    public void setEnabled(boolean isEnabled) {
        this.isEnabled = isEnabled;

        if(!isEnabled) stop();
    }

    private double buzzTime;
    /**
     * Returns the length of a buzz.
     * @return The length of a buzz, in seconds.
     */
    public double getBuzzTime() {
        return buzzTime;
    }
    /**
     * Sets the length of a buzz.
     * @param buzzTime The desired length of a buzz, in seconds.
     */
    public void setBuzzTime(double buzzTime) {
        this.buzzTime = buzzTime;
    }
    
    /**
     * Constructs a new Rumble.
     * <p>The Rumble class facilitates controller rumble and allows the user to schedule its actions.
     * @param controller The Joystick to rumble.
     * @param rumbleType The RumbleType of the Rumble. On Xbox controllers, kLeft is strong and kRight is light.
     * @param power The intensity of the Rumble out of 1.0.
     */
    public Rumble(Joystick controller, RumbleType rumbleType, double power) {
        this.controller = controller;
        this.rumbleType = rumbleType;
        this.power = power;

        buzzTimer = new Timer();

        isRumbling = false;
        isEnabled = true;
        buzzTime = 0.25;
    }

    /**
     * Starts the Rumble, if enabled.
     */
    public void start() {
        if(isEnabled) controller.setRumble(rumbleType, power);
    }

    /**
     * Stops the Rumble.
     */
    public void stop() {
        controller.setRumble(rumbleType, 0.0);
    }

    /**
     * Controller rumbles for 0.25 seconds, unless set for a different length.
     * <p>To change the length, use setBuzzTime.
     */
    public void buzz() {
        buzzTimer.start();
        start();
        Trigger stopper = new Trigger(() -> buzzTimer.hasElapsed(buzzTime));
        stopper.onTrue(new RunCommand(() -> stop()));
    }

    /**
     * Schedules the Rumble to start based on the boolean condition.
     * @param condition The condition that should be true to start the Rumble.
     */
    public void startWhen(BooleanSupplier condition) {
        Trigger scheduler = new Trigger(condition);
        scheduler.onTrue(new RunCommand(() -> start()));
    }

    /**
     * Schedules the Rumble to stop based on the boolean condition.
     * @param condition The condition that should be true to stop the Rumble.
     */
    public void stopWhen(BooleanSupplier condition) {
        Trigger scheduler = new Trigger(condition);
        scheduler.onTrue(new RunCommand(() -> stop()));
    }

    /**
     * Schedules the Rumble to start when the condition is true and stop when it is false.
     * @param condition The condition that should be true to start the Rumble.
     */
    public void rumbleWhile(BooleanSupplier condition) {
        Trigger scheduler = new Trigger(condition);
        scheduler.whileTrue(new RunCommand(() -> start())).whileFalse(new RunCommand(() -> stop()));
    }

    /**
     * Schedules the Rumble to buzz based on the boolean condition.
     * @param condition The condition that should be true to buzz.
     */
    public void buzzWhen(BooleanSupplier condition) {
        Trigger scheduler = new Trigger(condition);
        scheduler.onTrue(new RunCommand(() -> buzz()));
    }
}
