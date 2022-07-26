# Generating Upper-Body Motion for Real-Time Characters Making theirWay through Dynamic Environments

![teaser](Docs/Images/interactions-1.jpg)

- [Generating Upper-Body Motion for Real-Time Characters Making theirWay through Dynamic Environments](#generating-upper-body-motion-for-real-time-characters-making-theirway-through-dynamic-environments)
  - [Introduction](#introduction)
  - [Real-time Terrain Deformation](#real-time-terrain-deformation)
  - [Instructions](#instructions)
  - [Results](#results)
  - [Citation](#citation)
  - [Links](#links)
  - [License](#license)


<a name="Introduction"></a>
## Introduction

This repository provides the codes used to reproduce the results shown in the following paper: **Generating Upper-Body Motion for Real-Time Characters Making theirWay through Dynamic Environments**. Eduardo Alvarado, Damien Rohmer, Marie-Paule Cani.

This system takes as input a character model, ie. a mesh geometry with a rigged skeleton being animated by an arbitrary kinematic animations (e.g., keyframed clip, MoCap data) in order to build a responsive, partial physically-based version of the input skeleton. This real-time model allows to genereate plausible upper-body interactions from single motion clips, regardless of the nature of the environment, including non-rigid obstacles such as vegetation.

In order to reproduce two-ways interactions between the upper-body and the environment, we rely on a hybrid method for character animation that couples a keyframed sequence with kinematic constraints (IK) and lightweight physics for each limb independently. The dynamic response of the character’s upper-limbs leverages antagonistic controllers, allowing us to tune tension/relaxation in the upper-body without diverging from the reference keyframe motion. A new sight model, controlled by procedural rules, enables high-level authoring of the way the character generates interactions by adapting its stiffness and reaction time. As results show, our real-time method
offers precise and explicit control over the character’s behavior and style, while seamlessly adapting to new situations. Our model is therefore well suited for gaming applications.


<p align="center">
  <img src="Docs/Gifs/with-without.gif" width="100%">
</p>
<p align="center"><em>Figure 1: Left: Baseline animation. Right: Ours.</em></p>

<p align="center">
  <img src="Docs/Gifs/bananas-interactions.gif" width="40%">
&nbsp; &nbsp;
  <img src="Docs/Gifs/flowers-interactions.gif" width="40%">
</p>

<p align="center">
  <img src="Docs/Gifs/lianas-interactions.gif" width="40%">
&nbsp; &nbsp;
  <img src="Docs/Gifs/plants-interactions.gif" width="40%">
</p>
<p align="center"><em>Figure 2: Examples of anticipation and two-ways interactions with different obstacles.</em></p>

<a name="steps"></a>
## Real-time Terrain Deformation

We propose a model for the forces that the character applies to the ground when its feet are in contact with it, based on its kinematics and the nature of the ground. The resulting interaction forces over time are used to compute a plausible ground deformation.

The static forces that the model exerts on the ground are estimated based on the character's mass *m*, the contact area between the feet and the ground and the balance described by the contribution of each foot to the character's weight. In addition, a dynamic force model during contact takes into consideration the force that each foot generates due to its change of momentum when it lands into the ground with certain velocity. In order to define the time needed for the character to be fully stopped by a given type of terrain, we introduce an external parameter called the *characteristic time τ*. Therefore, a given forward kinematics motion provided as input can be associated to different forces, ie. a large magnitude of momentum force on a hard terrain with small *τ* value, and small force magnitude with long effect on a soft terrain with large *τ*.

<p align="center">
  <img src="Docs/Images/forces.png" width="100%">
</p>
<p align="center"><em>Figure 2: Forces generated during the kinematic motion. Blue: weight - Red: momentum forces - Black: foot-to-ground forces.</em></p>

Finally, we use a linear plastic model for terrain compression along with a ray-casting method to map the estimated forces into the respective ground deformation. Parameters such as the Young Modulus of Elasticity *E* or Poisson ratio *ν* can be modified to change the behavior of the terrain under deformation.

*For more information about the method and mathematical background behind the approach, please refer to the paper.*

<a name="Instructions"></a>
## Instructions

The repository contains all the necessary assets to run the project without additional material. The last version has been tested on the **Unity version 2020.3.21f1**. Inside the `Assets`, the following structure is introduced:

    .
    ├── ...
    ├── Assets 
    │   ├── ...		
    │   ├── Materials           # Materials used for models/grounds
    │   ├── Models              # Character models containing animation clips, state-machines or rigs
    │   ├── Scenes              # Scenes ready-to-use
    │   ├── Scripts             # .cs scripts for terrain deformation
    │   ├── Terrain Data        # Data files corresponding to terrain heightmaps
    │   ├── Textures            # Textures used for models/grounds
    │   └── ...                
    ├── Docs
    ├── ...				
    ├── README.md
    └── LICENSE

Go to `Assets > Scenes` and open the `Terrain Deformation` scene. Click in the **play button**, and after that, **select** the Game Object `Terrain` in the Hierarchy **to trigger on the deformation system**. In the `Game` window, you will find an environment where you can move your character, along with an interface to modify the terrain deformation parameters.

<p align="center">
  <img src="Docs/Images/unity-demo.png" width="100%">
</p>
<p align="center"><em>Figure 3: Demo interface for terrain deformation.</em></p>

In order to start deforming the terrain while you move, make sure that the option `Deformation` in the upper-left corner is active. If you want to activate the lateral dispacement, select the opcion `Bump`. To activate the post-processing step using Gaussian Blur, activate `Post-processing`. The options are:

* **Terrain Options**:
	* **Young's Modulus *E***: Defines the elastic constant of the terrain, measured in [Pa]. It has a direct impact on the terrain compressibility. Smaller values lead to more compressible terrains, and therefore deeper deformations.
	* **Contact Time *τ***: Defines the characteristic time of the terrain, measure in [s]. It is the time needed for the character to be fully stopped by a given type of terrain, during which the ground exerts a reaction force. Larger values lead to smaller force values during a larger period of time, while smaller values lead to stronger reaction forces during a short period of time.
	* **Poisson Ratio *ν***: Defines the amount of volume preserved at the end of the deformation, and is adimensional. An ideal incompressible material has a Poisson ratio of 0.5. This will cause a larger bump while the option is active, as the volume during the deformation is totally preserved. As the value of *ν* gets smaller, the material compresses and the amount of volume displaced on the outline of the footprint will be lower.
	* **Gaussian Iteratons**: Number of iterations for the Gaussian Blur filter. Larger values lead to smoother footprint results.
* **Show**:
	* **Raycast Grid**: Displays raycast method to estimate the contact area between the ground and the feet.
	* **Bump**: Displays contour of the footprint.
	* **Wireframe**: Switches to wireframe mode to show the 3D terrain mesh and the deformations.
	* **Forces**: Displays force model from the kinematic animation in real-time.
* **Forces**: Prints the values for weight, momentum forces and ground reaction forces per foot. Measured in [N].
* **Deformation**: Prints the values for the feet positions, number of ray hits used to calculate the contact area, pressure per foot measured in [N/m²] and compressive displacement per foot measured in [mm].

In order to reset the terrain, just select `Terrain` in the Hierarchy and use the `Set Height` brush in the `Terrain Component` in the Inspector. Then, set the `Height` value to 1 and paint the terrain to reset it (while the application is not playing).

In the folder `Scenes`, you can find the scene `Terrain Deformation - Showcases`. This scene contains three types of grounds (snow, sand and mud), defined by different parametrization. When the character moves though these terrains, the system will switch automatically based on the material where the character is stepping in, changing the deformation appearance.

*For more information about the values for each type of material, please refer to the paper.*

<a name="Results"></a>
## Results

<p align="center">
  <img src="Docs/Images/terrains.png" width="100%">
</p>

<p align="center">
  <img src="Docs/Gifs/quad.gif" width="60%">
</p>

<p align="center">
  <img src="Docs/Gifs/basic-footprints.gif" width="60%">
</p>

<a name="Citation"></a>
## Citation

```bibtex
@article{10.3389/frvir.2022.801856,
title = {Real-Time Locomotion on Soft Grounds With Dynamic Footprints},
author = {Alvarado, Eduardo and Paliard, Chloé and Rohmer, Damien and Cani, Marie-Paule},
doi = {10.3389/FRVIR.2022.801856},
journal = {Frontiers in Virtual Reality},
volume = {3},
pages = {3},
year = {2022},
month = {mar},
copyright = {All rights reserved},
issn = {2673-4192},
url = {https://hal.inria.fr/hal-03630136},
urldate = {2022-04-05}
}
```
<a name="Links"></a>
## Links

- [Project Page](https://edualvarado.com/real-time-locomotion-on-soft-grounds/)
- [Paper](https://hal.inria.fr/hal-03630136)
- [Video](https://www.youtube.com/watch?v=aWBntnCOwEE)

<a name="License"></a>
## License

The code is released under MIT License. See LICENSE for details.
