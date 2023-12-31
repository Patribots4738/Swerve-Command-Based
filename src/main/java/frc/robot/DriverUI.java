package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import io.github.oblarg.oblog.annotations.Log.BooleanBox;

public class DriverUI implements Loggable {

    // Sorry for the lack of comments in this file, I'm not sure what to say...
    // Preview: https://cdn.discordapp.com/attachments/445437792344866837/1092005173112619058/javaw_Rd0LNyEiIZ.gif
    // Be sure to look through https://oblog-docs.readthedocs.io/en/latest :')

    @Log(name="Select Auto:", rowIndex = 0, columnIndex = 4, height = 1, width = 2)
    public static SendableChooser<PathPlannerTrajectory> autoChooser = new SendableChooser<>();
    
    @BooleanBox(name = "Fresh Code", rowIndex = 0, columnIndex = 10, height = 1, width = 1, colorWhenTrue = "lime")
    public static boolean freshCode = true;

    @BooleanBox(name = "Enabled?", rowIndex = 0, columnIndex = 12, height = 1, width = 1, colorWhenTrue = "lime")
    public static boolean enabled = false;

    @BooleanBox(name = "Align Status", rowIndex = 0, columnIndex = 6, height = 1, width = 4, colorWhenTrue = "lime")
    public static boolean aligned = false;

    @BooleanBox(name = "Target Found", rowIndex = 0, columnIndex = 0, height = 1, width = 4, colorWhenTrue = "lime")
    public static boolean hasTargets = false;

    @Log.Field2d(name = "Field", rowIndex = 1, columnIndex = 0, height = 4, width = 10)
    public static Field2d field = new Field2d();

    @BooleanBox(name = "Cone Mode", rowIndex = 1, columnIndex = 10, height = 5, width = 3, 
                colorWhenTrue = "yellow", colorWhenFalse = "purple")
    public static boolean coneMode = false;

    @BooleanBox(name = "Spicy Claw", rowIndex = 5, columnIndex = 2, height = 1, width = 3)
    public static boolean spicyClaw = false;

    @Log(name = "Starting rotation", rowIndex = 5, columnIndex = 0, height = 1, width = 1)
    public static double startingRotation = 0;

    public DriverUI() {}
}
