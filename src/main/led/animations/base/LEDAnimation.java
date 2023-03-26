package frc.robot.led.animations.base;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.led.LEDStrip;

public abstract class LEDAnimation extends CommandBase {

    private final LEDStrip ledStrip;
    public LEDStrip getLEDStrip() {
        return ledStrip;
    }

    private int cycles = 0;
    public int getCyclesRun() {
        return cycles;
    }

    private final Timer timer;
    public double getTimeRun() {
        return timer.get();
    }

    private BooleanSupplier timeCondition;
    private BooleanSupplier cyclesCondition;
    private BooleanSupplier endCondition;

    protected final BooleanSupplier cycleCondition;
    private boolean hasCycled = false;

    public LEDAnimation(LEDStrip ledStrip) {
        this(ledStrip, () -> true);
    }

    public LEDAnimation(LEDStrip ledStrip, BooleanSupplier cycleCondition) {
        this.ledStrip = ledStrip;
        this.cycleCondition = cycleCondition;
        timer = new Timer();

        timeCondition = () -> false;
        cyclesCondition = () -> false;
        endCondition = () -> false;
    }

    public LEDAnimation runForever() {
        timeCondition = () -> false;
        cyclesCondition = () -> false;
        endCondition = () -> false;
        return this;
    }

    public LEDAnimation runForTime(double seconds) {
        timeCondition = () -> timer.hasElapsed(seconds);
        return this;
    }

    public LEDAnimation runForCycles(int times) {
        cyclesCondition = () -> cycles >= times;
        return this;
    }

    public LEDAnimation runForCondition(BooleanSupplier endCondition) {
        this.endCondition = endCondition;
        return this;
    }

    public boolean hasElapsed(double seconds) {
        return timer.hasElapsed(seconds);
    }

    public boolean hasCycled(int times) {
        return cycles >= times;
    }

    public abstract void run();

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        if(cycleCondition.getAsBoolean() && !hasCycled) {
            cycles++;
            hasCycled = true;
        }
        else if(!cycleCondition.getAsBoolean()) {
            hasCycled = false;
        }
        run();
    }

    @Override
    public void end(boolean interrupted) {
        cycles = 0;
        hasCycled = false;
        ledStrip.cancelAnimation();
    }

    @Override
    public boolean isFinished() {
        return timeCondition.getAsBoolean() || cyclesCondition.getAsBoolean() || endCondition.getAsBoolean();
    }
}
