# FRC 4152 Rick

### 2024 Robot With AdvantageKit and MapleSim Simulation

![Robot Image](/assets/_MG_2139.jpg)

---

Swerve based on original project [here](https://www.chiefdelphi.com/t/advantagekit-2024-log-replay-again/442968/54).

## Highlights
- 5 note auto
- 7 Best Endgame EPA on statbotics

## Using Simulation
When running the simulation, you can see the simulation results through AdvantageScope

- Add `/Robot_Rick_2024` to AdvantageScope assets folder
- Add `/Robot_Rick_Simple_2024` to AdvantageScope assets folder
- Ensure `/src/java/frc/robot/Constants.java` currentMode is set to Mode.SIM
- Run code in simulation.

Please drag the following fields:

- Open field `AdvantageKit/RealOutputs/FieldSimulation/RobotPosition` make it `Robot/Rick 2024`
- Open field `AdvantageKit/RealOutputs/Intake/IntakePose` make it component 1 of `Robot/Rick 2024`
- Open field `AdvantageKit/RealOutputs/Climber/ClimberPose` make it component 2 of `Robot/Rick 2024`
- Open field `AdvantageKit/RealOutputs/ElevatorTrap/ElevatorStage2Pose` make it component 3 of `Robot/Rick 2024`
- Open field `AdvantageKit/RealOutputs/ElevatorTrap/ElevatorStage3Pose` make it component 4 of `Robot/Rick 2024`
- Open field `AdvantageKit/Vision/TagPoses` make it a vision target of `Robot/Rick 2024`
- Open field `AdvantageKit/RealOutputs/FieldSimulation/Notes` and make it `Note`
- Open field `AdvantageKit/RealOutputs/Intake/NoteInIntake` and make it `Note`
- Open field `AdvantageKit/RealOutputs/Odometry/Robot` and make it `Green Ghost/Rick Simple 2024`
- Open field `AdvantageKit/Vision/Pose` and make it `Blue Ghost/Rick Simple 2024`

## Controls
![Controller Image](/assets/XboxDiagram.png)
Comming...