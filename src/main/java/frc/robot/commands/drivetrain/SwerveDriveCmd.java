package frc.robot.commands.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSys;

public class SwerveDriveCmd extends CommandBase {

    /**
     * Command to allow for driver input in teleop
     * Can't be inlined efficiently if we want to edit the inputs in any way (deadband, square, etc.)
     */

    private final SwerveSys driveSys;

    /**
     * Joysticks return DoubleSuppliers when the get methods are called
     * This is so that joystick getter methods can be passed in as a parameter but will continuously update, 
     * versus using a double which would only update when the constructor is called
     */
    private final DoubleSupplier drive;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rot;

    private final BooleanSupplier lock;
    
    private final boolean isFieldRelative;


    public SwerveDriveCmd(
        SwerveSys driveSys, 
        DoubleSupplier drive, 
        DoubleSupplier strafe, 
        DoubleSupplier rot,
        BooleanSupplier lock,
        boolean isFieldRelative
    ) {

        this.driveSys = driveSys;

        this.drive = drive;
        this.strafe = strafe;
        this.rot = rot;

        this.lock = lock;

        this.isFieldRelative = isFieldRelative;

        addRequirements(driveSys);

    }
    
    @Override
    public void execute() {

        /**
         * Units are given in meters per second radians per second
         * Since joysticks give output from -1 to 1, we multiply the outputs by the max speed
         * Otherwise, our max speed would be 1 meter per second and 1 radian per second
         */

        double drive = this.drive.getAsDouble();
        drive = deadbandInputs(drive);
        drive *= Math.abs(drive);

        double strafe = this.strafe.getAsDouble();
        strafe = deadbandInputs(strafe);
        strafe *= Math.abs(strafe);

        double rot = this.rot.getAsDouble();
        rot = deadbandInputs(rot);
        rot *= Math.abs(rot);

        boolean lock = this.lock.getAsBoolean();

        driveSys.drive(
            -drive,
            -strafe,
            -rot,
            lock,
            isFieldRelative
        );

    }

    // method to deadband inputs to eliminate tiny unwanted values from the joysticks
    public double deadbandInputs(double input) {

        if (Math.abs(input) < Constants.Controllers.operatorControllerDeadband)
            return 0.0;
        
        return input;

    }

}