package frc.robot;

import java.util.Arrays;
import java.util.List;

import frc.subsystems.Subsystem;


public class SubsystemManager {
    private static SubsystemManager Instance;

    private List<Subsystem> allSubsystems;
    private List<Subsystem> debugSubsystems;

    private SubsystemManager() {}

    public void registerSubsystems(Subsystem... subsystems)
    {
        allSubsystems = Arrays.asList(subsystems);
    }

    public void registerDebugSubsystems(Subsystem... subsystems)
    {
        debugSubsystems = Arrays.asList(subsystems);
    }

    public void smartDashboard()
    {
        allSubsystems.forEach(Subsystem::smartDashboard);
    }

    public void smartDashboard_DEBUG()
    {
        debugSubsystems.forEach(Subsystem::smartDashboard_DEBUG);
    }

    public void update()
    {
        allSubsystems.forEach(Subsystem::update);
    }

    public void start()
    {
        allSubsystems.forEach(Subsystem::onStart);
    }

    public void stop()
    {
        allSubsystems.forEach(Subsystem::stop);
    }

    public static SubsystemManager getInstance()
    {
        if(Instance == null)
        {
            Instance = new SubsystemManager();
        }
        return Instance;
    }
}
