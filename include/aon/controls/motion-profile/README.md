# Motion Profile

## Theory

You can use the following resources to start getting familiarized with motion profiles. I specially recommend the FIRST Robotics video, which was the most valuable resource when I started learning about Motion Profiles.

[AON Robotics - Introduction to Motion Profile.pdf](./Introduction%20to%20Motion%20Profile.pdf)

[ACME Robotics - Motion Profiling](https://acme-robotics.gitbook.io/road-runner/tour/motion-profiling)

[Motion Control Tips - What is motion profile?]( https://www.motioncontroltips.com/what-is-a-motion-profile/)

[Okapilib - 2D Motion Profiling](https://okapilib.github.io/OkapiLib/md_docs_tutorials_concepts_twodmotionprofiling.html)

[YouTube - 2015 FIRST World Championships Conference - Motion Planning & Control in FRC ](https://youtu.be/8319J1BEHwM)

## My modifications

Since I don't have much experience with path planning or trayectory planning, I kept the algorithms simple. I am working with 1D motion profiles and I used an inverse position profile as a measure of "progress". See [Introduction to Motion Profile.pdf](./Introduction%20to%20Motion%20Profile.pdf) for more info. You can check the AON Robotics 2021 - Tipping Point repository (include/aon/controls/path-planning) to see what progress I made regarding path planning using Hermite splines. However, the code was problematic and I decided to keep it simple due to lack of time.

## What's in this folder?

In this folder we have:
1. [motion-profile.hpp](./motion-profile.hpp): This is the base class for everything else that will be implemented.
2. [trapezoid-profile.hpp](./trapezoid-profile.hpp): This is the "main" motion profile we will use for drive motions.
3. [exponential-profile.hpp](./exponential-profile.hpp): This motion profile is specially useful for pure rotations.
4. [quintic-profile.hpp](./quintic-profile.hpp): This motion profile might be useful for rotations while the robot is also moving.
5. [Introduction to Motion Profile.pdf](./Introduction%20to%20Motion%20Profile.pdf): Document that explains motion profiles and derives the trapezoidal motion profile.
6. [TrapezoidProfile.m](./TrapezoidProfile.m): MATLAB script that uses the Symbolic Toolbox to help derive the trapezoidal motion profile.