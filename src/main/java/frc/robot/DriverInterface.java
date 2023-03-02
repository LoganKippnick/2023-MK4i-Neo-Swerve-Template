package frc.robot;

import java.util.Set;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.LiftSys;
import frc.robot.subsystems.SwerveSys;
import frc.robot.subsystems.VisionSys;

public class DriverInterface extends SubsystemBase {

    private final ShuffleboardTab tab;

    private final Field2d field;

    private final SwerveSys swerveSys;
    private final LiftSys liftSys;
    private final ClawSys clawSys;
    private final IntakeSys intakeSys;
    private final VisionSys visionSys;

    public DriverInterface(SwerveSys swerveSys, LiftSys liftSys, ClawSys clawSys, IntakeSys intakeSys, VisionSys visionSys) {
        this.swerveSys = swerveSys;
        this.liftSys = liftSys;
        this.clawSys = clawSys;
        this.intakeSys = intakeSys;
        this.visionSys = visionSys;

        tab = Shuffleboard.getTab("driver interface");

        field = new Field2d();
    }

    @Override
    public void periodic() {
        field.setRobotPose(swerveSys.getPose());
        tab.add(field);
    } 
}
