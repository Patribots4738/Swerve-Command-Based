package frc.robot;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import monologue.Monologue.LogNT;
import monologue.Logged;

public class DriverUI implements Logged {

    // Sorry for the lack of comments in this file, I'm not sure what to say...
    // Preview: https://cdn.discordapp.com/attachments/445437792344866837/1092005173112619058/javaw_Rd0LNyEiIZ.gif
    // Be sure to look through https://oblog-docs.readthedocs.io/en/latest :')

    @LogNT
    public static SendableChooser<PathPlannerTrajectory> autoChooser = new SendableChooser<>();
    
    @LogNT
    public static boolean freshCode = true;

    @LogNT
    public static Field2d field = new Field2d();

    public DriverUI() {}
}
