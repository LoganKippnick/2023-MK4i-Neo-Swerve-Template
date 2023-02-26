package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Rumble {

    private final GenericHID[] controllers;
    private final RumbleType rumbleType;
    private final double power;

    private final CommandBase debug;

    private double pulseLength = 0.25;
    private double pulseTime = 0.5;

    public double getPulseLength() {
        return pulseLength;
    }
    public void setPulseLength(double pulseLength) {
        this.pulseLength = pulseLength;
    }
    public double getPulseTime() {
        return pulseTime;
    }
    public void setPulseTime(double pulseTime) {
        if(pulseTime < pulseLength) pulseTime = pulseLength;
        else this.pulseTime = pulseTime;
    }

    private boolean isEnabled;
    private boolean isRumbling;
    private boolean isPulsing;

    public boolean isEnabled() {
        return isEnabled;
    }
    public void setEnabled(boolean isEnabled) {
        this.isEnabled = isEnabled;
    }
    public boolean isRumbling() {
        return isRumbling;
    }
    public boolean isPulsing() {
        return isPulsing;
    }

    public Rumble(RumbleType rumbleType, double power, GenericHID... controllers) {
        this.rumbleType = rumbleType;
        this.power = power;
        this.controllers = controllers;

        isEnabled = true;
        isRumbling = false;
        isPulsing = false;

        debug = new CommandBase() {
            @Override
            public void execute() {
                SmartDashboard.putBoolean("isRumbling", isRumbling);
                SmartDashboard.putBoolean("isPulsing", isPulsing);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    public void debug() {
        CommandScheduler.getInstance().schedule(debug);
    }

    public void startRumble() {
        CommandBase startRumble = new CommandBase() {
            @Override
            public void initialize() {
                isRumbling = true;
            }

            @Override
            public void execute() {
                if(isEnabled) {
                    for(GenericHID controller : controllers) controller.setRumble(rumbleType, power);
                }
                else {
                    for(GenericHID controller : controllers) controller.setRumble(rumbleType, 0.0);
                }
            }

            @Override
            public void end(boolean interrupted) {
                for(GenericHID controller : controllers) controller.setRumble(rumbleType, 0.0);
            }

            @Override
            public boolean isFinished() {
                return !isRumbling;
            }
        };
        CommandScheduler.getInstance().schedule(startRumble);
    }

    public void startRumbleWhen(BooleanSupplier condition) {
        CommandBase rumbleStarter = new CommandBase() {
            @Override
            public void execute() {
                if(condition.getAsBoolean() && !isRumbling) startRumble();
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        CommandScheduler.getInstance().schedule(rumbleStarter);
    }

    public void stopRumble() {
        isRumbling = false;
    }

    public void stopRumbleWhen(BooleanSupplier condition) {
        CommandBase rumbleStopper = new CommandBase() {
            @Override
            public void execute() {
                if(condition.getAsBoolean() && isRumbling) stopRumble();
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        CommandScheduler.getInstance().schedule(rumbleStopper);
    }

    public void rumble(double seconds) {
        CommandBase pulse = new CommandBase() {
            Timer timer;

            @Override
            public void initialize() {
                timer = new Timer();
                startRumble();
                timer.start();
            }

            @Override
            public void end(boolean interrupted) {
                stopRumble();
                timer.stop();
            }

            @Override
            public boolean isFinished() {
                return timer.hasElapsed(seconds);
            }
        };
        CommandScheduler.getInstance().schedule(pulse);
    }

    public void rumbleWhen(BooleanSupplier condition, double seconds) {
        CommandBase rumbleScheduler = new CommandBase() {
            @Override
            public void execute() {
                if(condition.getAsBoolean() && !isRumbling) rumble(seconds);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        CommandScheduler.getInstance().schedule(rumbleScheduler);
    }

    public void rumbleWhile(BooleanSupplier condition) {
        CommandBase rumbleScheduler = new CommandBase() {
            @Override
            public void execute() {
                if(!isRumbling && condition.getAsBoolean()) {
                    startRumble();
                }
                else if(isRumbling && !condition.getAsBoolean()) {
                    stopRumble();
                }
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        CommandScheduler.getInstance().schedule(rumbleScheduler);
    }

    public void startPulse() {
        CommandBase startPulse = new CommandBase() {
            Timer timer;

            @Override
            public void initialize() {
                isPulsing = true;
                timer = new Timer();
                startRumble();
                timer.start();
            }

            @Override
            public void execute() {
                if(isRumbling && timer.hasElapsed(pulseLength)) {
                    stopRumble();
                }
                else if(!isRumbling && timer.hasElapsed(pulseTime)) {
                    startRumble();
                    timer.reset();
                }
            }

            @Override
            public void end(boolean interrupted) {
                stopRumble();
                timer.stop();
            }

            @Override
            public boolean isFinished() {
                return !isPulsing;
            }
        };
        CommandScheduler.getInstance().schedule(startPulse);
    }

    public void startPulseWhen(BooleanSupplier condition) {
        CommandBase pulseStarter = new CommandBase() {
            @Override
            public void execute() {
                if(condition.getAsBoolean() && !isPulsing) startPulse();
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        CommandScheduler.getInstance().schedule(pulseStarter);
    }

    public void stopPulse() {
        isPulsing = false;
    }

    public void stopPulseWhen(BooleanSupplier condition) {
        CommandBase pulseStopper = new CommandBase() {
            @Override
            public void execute() {
                if(condition.getAsBoolean() && isPulsing) stopPulse();
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        CommandScheduler.getInstance().schedule(pulseStopper);
    }

    public void pulse() {
        CommandBase pulseOnce = new CommandBase() {
            Timer timer;

            @Override
            public void initialize() {
                timer = new Timer();
                isPulsing = true;
                startRumble();
                timer.start();
            }

            @Override
            public void end(boolean interrupted) {
                isPulsing = false;
                stopRumble();
                timer.stop();
            }

            @Override
            public boolean isFinished() {
                return timer.hasElapsed(pulseLength);
            }
        };
        CommandScheduler.getInstance().schedule(pulseOnce);
    }
    
    public void pulse(double times) {
        CommandBase pulseTimes = new CommandBase() {
            Timer timer;
            double pulseCount = 0;

            @Override
            public void initialize() {
                timer = new Timer();
                startRumble();
                isPulsing = true;
                timer.start();
            }

            @Override
            public void execute() {
                if(isRumbling && timer.hasElapsed(pulseLength)) {
                    stopRumble();
                }
                else if(!isRumbling && timer.hasElapsed(pulseTime)) {
                    startRumble();
                    pulseCount++;
                    timer.reset();
                }
            }

            @Override
            public void end(boolean interrupted) {
                stopRumble();
                isPulsing = false;
                timer.stop();
            }

            @Override
            public boolean isFinished() {
                return pulseCount >= times || !isEnabled;
            }
        };
        CommandScheduler.getInstance().schedule(pulseTimes);
    }

    public void pulseWhen(BooleanSupplier condition) {
        CommandBase rumbleStarter = new CommandBase() {
            boolean hasPulsed = false;

            @Override
            public void execute() {
                if(condition.getAsBoolean() && !isPulsing  && !hasPulsed) {
                    pulse();
                    hasPulsed = true;
                }
                
                if(!condition.getAsBoolean() && !isPulsing) {
                    hasPulsed = false;
                }
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        CommandScheduler.getInstance().schedule(rumbleStarter);
    }

    public void pulseWhen(BooleanSupplier condition, int times) {
        CommandBase rumbleStarter = new CommandBase() {
            boolean hasPulsed = false;

            @Override
            public void execute() {
                if(condition.getAsBoolean() && !isPulsing  && !hasPulsed) {
                    pulse(times);
                    hasPulsed = true;
                }
                
                if(!condition.getAsBoolean() && !isPulsing) {
                    hasPulsed = false;
                }
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        CommandScheduler.getInstance().schedule(rumbleStarter);
    }

    public void pulseWhile(BooleanSupplier condition) {
        CommandBase pulseScheduler = new CommandBase() {
            @Override
            public void execute() {
                if(!isPulsing && condition.getAsBoolean()) {
                    startPulse();
                }
                else if(isPulsing && !condition.getAsBoolean()) {
                    stopPulse();
                }
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        CommandScheduler.getInstance().schedule(pulseScheduler);
    }

}