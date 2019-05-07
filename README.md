# Segcycle
Preliminary analysis of the dynamics and control of the Segcycle.

Analysis was first done in 2D, looking at the aerial-view dynamics of the bicycle treating the motion of each wheel as a non-holonomic constraint at the contact point with the ground. Trajectories for the bicycle were created from arbitrary inputs for the steering angles. It was shown that sinusoidal steering inputs gave quasi-periodic trajectories for the bicycle in the plane.

Continued analysis was done in 3D, looking now at the full-system. Each wheel is treated as with infinitessimal radius, thus effectively as a non-holonomic constraint at the contact point with the ground. The movement of this contact point created when the bicycle is leaned and steered is not considered because of the assumption that the wheel is infinitessimally small. Trajectories for the bicycle were created from arbitrary inputs for the steering angles, but a controller was created to make sure the bicycle prioritized stability.
