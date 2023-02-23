package frc.robot;

import java.util.concurrent.atomic.AtomicInteger;
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
    private final Timer pulseTimer;

    private boolean isRumbling;
    /**
     * Returns whether the controller is rumbling.
     * @return True if the controller is rumbling.
     */
    public boolean isRumbling() {
        return isRumbling;
    }

    private boolean isPulsing;
    /**
     * Returns whether the controller is pulsing.
     * @return True if the controller is pulsing.
     */
    public boolean isPulsing() {
        return isPulsing;
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
     * Sets whether the Rumble is enabled. If false, stops the rumbling and pulsing instantly.
     * @param isEnabled True if the Rumble should be enabled, false if it should be disabled.
     */
    public void setEnabled(boolean isEnabled) {
        this.isEnabled = isEnabled;

        if(!isEnabled) stop();
    }

    private double pulseTime;
    /**
     * Returns the length of a pulse.
     * @return The length of a pulse, in seconds.
     */
    public double getPulseTime() {
        return pulseTime;
    }
    /**
     * Sets the length of a pulse.
     * @param pulseTime The desired length of a pulse, in seconds.
     */
    public void setPulseTime(double pulseTime) {
        this.pulseTime = pulseTime;
    }

    private double pulseGapTime = 0.33; // TODO: Javadoc
    public double getPulseGapTime() {
        return pulseGapTime;
    }
    public void setPulseGapTime(double pulseGapTime) {
        this.pulseGapTime = pulseGapTime;
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
        
        pulseTimer = new Timer();

        isRumbling = false;
        isPulsing = false;
        isEnabled = true;
        pulseTime = 0.25;
    }

    /**
     * Starts the Rumble, if enabled.
     */
    public void start() {
        isRumbling = true;
        if(isEnabled) controller.setRumble(rumbleType, power);
    }

    /**
     * Stops the Rumble.
     */
    public void stop() {
        isRumbling = false;
        controller.setRumble(rumbleType, 0.0);
    }

    /**
     * Controller rumbles for a short period of time once.
     * <p>To change the length of a pulse, use setPulseTime.
     */
    public void pulse() {
        isPulsing = true;
        pulseTimer.start();
        start();
        Trigger stopper = new Trigger(() -> pulseTimer.hasElapsed(pulseTime));
        stopper.onTrue(new RunCommand(() -> {
            stop();
            stopPulse();
        }));
    }

    public void startPulse() { // TODO: Javadoc
        isPulsing = true;
        pulseTimer.start();
        Trigger stopper = new Trigger(() -> pulseTimer.hasElapsed(pulseTime));
        stopper.onTrue(new RunCommand(() -> stop()));
        Trigger waiter = new Trigger(() -> pulseTimer.hasElapsed(pulseTime + pulseGapTime));
        waiter.onTrue(new RunCommand(() -> {
            if(isPulsing) start();
            pulseTimer.reset();
        }));
    }

    public void stopPulse() { // TODO: Javadoc
        isPulsing = false;
        pulseTimer.stop();
        pulseTimer.reset();
    }

    /**
     * Controller rumbles for a short period of time a specified number of times.
     * <p>To change the length of a pulse, use setPulseTime. To change the time between pusles, use setPulseGapTime.
     * @param times The desired number of pulses.
     */
    public void pulse(int times) {
        isPulsing = true;
        pulseTimer.start();
        start();
        for(AtomicInteger i = new AtomicInteger(0); i.get() < times; i.incrementAndGet()) {
            Trigger stopper = new Trigger(() -> pulseTimer.hasElapsed(((pulseTime + pulseGapTime) * i.get()) + pulseTime));
            stopper.onTrue(new RunCommand(() -> stop()));
            if(i.get() < times - 1) {
                Trigger waiter = new Trigger(() -> pulseTimer.hasElapsed((pulseTime + pulseGapTime) * (i.get() + 1)));
                waiter.onTrue(new RunCommand(() -> start()));
            }
            else {
                stopper.onTrue(new RunCommand(() -> stopPulse()));
            }
        }
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
        scheduler.onTrue(new RunCommand(() -> start())).onFalse(new RunCommand(() -> stop()));
    }

    /**
     * Schedules the Rumble to start pulsing based on the boolean condition.
     * @param condition The condition that should be true to start pulsing.
     */
    public void startPulseWhen(BooleanSupplier condition) {
        Trigger scheduler = new Trigger(condition);
        scheduler.onTrue(new RunCommand(() -> startPulse()));
    }

    /**
     * Schedules the Rumble to stop pulsing based on the boolean condition.
     * @param condition The condition that should be true to stop pulsing.
     */
    public void stopPulseWhen(BooleanSupplier condition) {
        Trigger scheduler = new Trigger(condition);
        scheduler.onTrue(new RunCommand(() -> stopPulse()));
    }

    /**
     * Schedules the Rumble to pulse once based on the boolean condition.
     * @param condition The condition that should be true to pulse once.
     */
    public void pulseWhen(BooleanSupplier condition) {
        Trigger scheduler = new Trigger(condition);
        scheduler.onTrue(new RunCommand(() -> pulse()));
    }

    /**
     * Schedules the Rumble to pulse a specified number of times based on the boolean condition.
     * @param condition The condition that should be true to pulse a specified number of times.
     * @param times The desired number of pulses.
     */
    public void pulseWhen(BooleanSupplier condition, int times) {
        Trigger scheduler = new Trigger(condition);
        scheduler.onTrue(new RunCommand(() -> pulse(times)));
    }

    /**
     * Schedules the Rumble to pulse when the condition is true and stop when it is false.
     * @param condition The condition that should be true to start pulsing.
     */
    public void pulseWhile(BooleanSupplier condition) {
        Trigger scheduler = new Trigger(condition);
        scheduler.onTrue(new RunCommand(() -> start())).onFalse(new RunCommand(() -> stop()));
    }
}
