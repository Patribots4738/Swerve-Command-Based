package frc.robot;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot.GameMode;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.drive.WheelRadiusCharacterization;
import frc.robot.commands.logging.NTGainTuner;
import frc.robot.commands.managers.HDCTuner;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.test.KrakenTest;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.OIConstants;
import frc.robot.util.auto.PathPlannerStorage;
import frc.robot.util.custom.PatriBoxController;
import frc.robot.util.hardware.rev.Neo;

public class RobotContainer {

    private PowerDistribution pdh;

    private EventLoop testButtonBindingLoop = new EventLoop();

    private final PatriBoxController driver;
    @SuppressWarnings("unused")
    private final PatriBoxController operator;

    private boolean fieldRelativeToggle = true;
    private final BooleanSupplier robotRelativeSupplier;

    private final Swerve swerve;
    private final KrakenTest krakenTest;

    public static Field2d field2d = new Field2d();

    private PathPlannerStorage pathPlannerStorage;
    private static HDCTuner HDCTuner;

    // Draggables
    @AutoLogOutput (key = "Draggables/FreshCode")
    public static boolean freshCode = true;
    @AutoLogOutput (key = "Draggables/RobotPose2d")
    public static Pose2d robotPose2d = new Pose2d();
    @AutoLogOutput (key = "Draggables/RobotPose3d")
    public static Pose3d robotPose3d = new Pose3d();
    @AutoLogOutput (key = "Draggables/SwerveMeasuredStates")
    public static SwerveModuleState[] swerveMeasuredStates;
    @AutoLogOutput (key = "Draggables/SwerveDesiredStates")
    public static SwerveModuleState[] swerveDesiredStates;
    @AutoLogOutput (key = "Draggables/GameModeStart")
    public static double gameModeStart = 0;
    
    public RobotContainer() {

        System.out.println("Constructing Robot Container...");

        driver = new PatriBoxController(OIConstants.DRIVER_CONTROLLER_PORT, OIConstants.DRIVER_DEADBAND);
        operator = new PatriBoxController(OIConstants.OPERATOR_CONTROLLER_PORT, OIConstants.OPERATOR_DEADBAND);

        pdh = new PowerDistribution(30, ModuleType.kRev);
        pdh.setSwitchableChannel(false);

        swerve = new Swerve();
        krakenTest = new KrakenTest();

        SmartDashboard.putData(field2d);

        driver.back().toggleOnTrue(
            Commands.runOnce(() -> fieldRelativeToggle = !fieldRelativeToggle)
        );
        robotRelativeSupplier = () -> fieldRelativeToggle;

        swerve.setDefaultCommand(new Drive(
            swerve,
            driver::getLeftY,
            driver::getLeftX,
            () -> -driver.getRightX()/1.6,
            robotRelativeSupplier,
            () -> (robotRelativeSupplier.getAsBoolean() && Robot.isRedAlliance())
        ));

        HDCTuner = new HDCTuner(
            AutoConstants.HDC.getXController(),
            AutoConstants.HDC.getThetaController());

        Neo.incinerateMotors();
        configureButtonBindings();
        configureTimedEvents();

        pathPlannerStorage = new PathPlannerStorage(swerve);

        pathPlannerStorage.configureAutoChooser();
        pathPlannerStorage.getAutoChooser().addOption("WheelRadiusCharacterization",
            swerve.setWheelsOCommand()
            .andThen(new WheelRadiusCharacterization(swerve)));

        new NTGainTuner().schedule();
        
        prepareNamedCommands();

    }

    private void configureButtonBindings(){
        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureTimedEvents() {}

    private void configureOperatorBindings() {}

    private void configureDriverBindings() {

        driver.start().onTrue(
            Commands.runOnce(() -> swerve.resetOdometry(
                new Pose2d(
                    swerve.getPose().getTranslation(), 
                    Rotation2d.fromDegrees(
                        Robot.isRedAlliance() 
                        ? 0 
                        : 180))
            ), swerve)
        );

        driver.povLeft()
            .onTrue(krakenTest.setVelocity(1000))
            .onFalse(krakenTest.setVelocity(0));

        driver.povUp()
            .onTrue(krakenTest.setPosition(500))
            .onFalse(krakenTest.setPosition(0));
        
        driver.povRight()
            .onTrue(krakenTest.setPercent(1.0))
            .onFalse(krakenTest.setPercent(0));

        driver.leftBumper().whileTrue(Commands.run(swerve::getSetWheelsX));
    }

    public void updateNTGains() {
        double HPFCP = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/0P").getDouble(-1);
        double HPFCI = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/1I").getDouble(-1);
        double HPFCD = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/2D").getDouble(-1);
        double HPFCP2 = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/0P").getDouble(-1);
        double HPFCI2 = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/1I").getDouble(-1);
        double HPFCD2 = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/2D").getDouble(-1);

        double HDCP = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Translation/0P").getDouble(-1);
        double HDCI = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Translation/1I").getDouble(-1);
        double HDCD = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Translation/2D").getDouble(-1);
        double HDCP2 = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Rotation/0P").getDouble(-1);
        double HDCI2 = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Rotation/1I").getDouble(-1);
        double HDCD2 = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Rotation/2D").getDouble(-1);

        if (HPFCP == -1 || HPFCI == -1 || HPFCD == -1 || HPFCP2 == -1 || HPFCI2 == -1 || HPFCD2 == -1 ||
            HDCP == -1 || HDCI == -1 || HDCD == -1 || HDCP2 == -1 || HDCI2 == -1 || HDCD2 == -1) {
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/0P").setDouble(AutoConstants.XY_CORRECTION_P);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/1I").setDouble(AutoConstants.XY_CORRECTION_I);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/2D").setDouble(AutoConstants.XY_CORRECTION_D);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/0P").setDouble(AutoConstants.ROTATION_CORRECTION_P);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/1I").setDouble(AutoConstants.ROTATION_CORRECTION_I);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/2D").setDouble(AutoConstants.ROTATION_CORRECTION_D);

            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Translation/0P").setDouble(AutoConstants.XY_CORRECTION_P);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Translation/1I").setDouble(AutoConstants.XY_CORRECTION_I);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Translation/2D").setDouble(AutoConstants.XY_CORRECTION_D);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Rotation/0P").setDouble(AutoConstants.ROTATION_CORRECTION_P);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Rotation/1I").setDouble(AutoConstants.ROTATION_CORRECTION_I);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Rotation/2D").setDouble(AutoConstants.ROTATION_CORRECTION_D);
            return;
        } else {
            AutoConstants.HPFC = new HolonomicPathFollowerConfig(
                    new PIDConstants(
                            HPFCP,
                            HPFCI,
                            HPFCD),
                    new PIDConstants(
                            HPFCP2,
                            HPFCI2,
                            HPFCD2),
                    DriveConstants.MAX_SPEED_METERS_PER_SECOND,
                    Math.hypot(DriveConstants.WHEEL_BASE, DriveConstants.TRACK_WIDTH) / 2.0,
                    new ReplanningConfig());

            AutoConstants.XY_PID.setP(HDCP);
            AutoConstants.XY_PID.setI(HDCI);
            AutoConstants.XY_PID.setD(HDCD);
            AutoConstants.THETA_PID.setP(HDCP2);
            AutoConstants.THETA_PID.setI(HDCI2);
            AutoConstants.THETA_PID.setD(HDCD2);

        }
    }

    public Command getAutonomousCommand() {
        return pathPlannerStorage.getSelectedAuto();
    }

    private void configureHDCBindings(PatriBoxController controller) {
        controller.pov(0, 270, testButtonBindingLoop)
            .onTrue(HDCTuner.controllerDecrementCommand());

        controller.pov(0, 90, testButtonBindingLoop)
            .onTrue(HDCTuner.controllerIncrementCommand());

        controller.pov(0, 0, testButtonBindingLoop)
            .onTrue(HDCTuner.increaseCurrentConstantCommand(.1));

        controller.pov(0, 180, testButtonBindingLoop)
            .onTrue(HDCTuner.increaseCurrentConstantCommand(-.1));

        controller.rightBumper(testButtonBindingLoop)
            .onTrue(HDCTuner.constantIncrementCommand());

        controller.leftBumper(testButtonBindingLoop)
            .onTrue(HDCTuner.constantDecrementCommand());

        controller.a(testButtonBindingLoop)
            .onTrue(HDCTuner.logCommand());

        controller.x(testButtonBindingLoop)
            .onTrue(HDCTuner.multiplyPIDCommand(2));

        controller.b(testButtonBindingLoop)
            .onTrue(HDCTuner.multiplyPIDCommand(.5));
    }

    public void onDisabled() {
        swerve.stopDriving();
        pathPlannerStorage.updatePathViewerCommand().schedule();
        pathPlannerStorage.configureAutoChooser();

        // TODO: Extract this into a command file
        Commands.run(this::updateNTGains)
            .until(() -> Robot.gameMode != GameMode.DISABLED)
            .ignoringDisable(true)
            .schedule();
    }

    public void onEnabled() {
        gameModeStart = Robot.currentTimestamp;
        pathPlannerStorage.updatePathViewerCommand().schedule();
        freshCode = false;
    }

    private void prepareNamedCommands() {}

}
