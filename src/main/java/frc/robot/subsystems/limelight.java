package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class limelight extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(LimelightConstants.Name);
    
    private final Field2d field2d;

    public limelight() {
        field2d = new Field2d();
    }

    
}
