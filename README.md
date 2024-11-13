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
### Driver Controller
- left Stick: Swerve XY Translation.
- Right Stick: Swerve Rotation.
- Left Bumper: Manual shoot will launch a note regardless of robots location and angle.
- Left Trigger: Auto shoot will automaticaly aim and drive the robot to the speaker and take the shot when all conditions are meet.
- Right Bumper: Runs intake at the floor pickup position with no assist.
- Left Bumper: Runs intake at the floor pickup position. If a note is visable then it will center the intake on the note. If the trigger is pressed over 50% then it will automaticaly drive the robot towards the visable note, varying the speed based on how much the trigger is pressed past the 50% point.
- left Stick Button (Our controller has back paddles mapped to the stick buttons): Automaticaly aim and lob a note to land infront of the speaker.
- Right Stick Button (Our controller has back paddles mapped to the stick buttons): Automaticaly drives to the amp and scores the amp once in location.
- A: Hold to shoot the note from against the subwoofer.
- B: Score the amp with manual alignment.
- X: Reset Pose Estimation.
- Y: Shoot note out of intake. Used mainly for a 2 notes stuck in the intake.