# An Event-Triggered Multi-UAV Coordination Scheme for Simultaneous Tracking and Pursuit of Multiple Moving Targets\
This paper proposes an event-triggered coordination scheme of multiple unmanned aerial vehicles (UAVs) for the purpose of tracking and monitoring multiple moving target vessels in maritime patrol scenario. 
In the proposed method, each UAV is assumed to be equipped with optical sensor which is used to measure its distances from the air to target vessels moving on water surface.
In particular, each UAV combines its obtained measurements with those of other UAVs to be subsequently processed using an event-triggered distributed extended Kalman filter algorithm to obtain its estimates of target vessels' positions. 
The estimated position of the target vessels are then used by the UAVs to construct dynamic convex hulls that connect all of the detected moving target vessels. 
The multi-UAV tracker system then generates spatio-temporal path curves using cycloid-type trajectories with a formation geometry that is adapted to cover the constructed convex hulls of the target vessels. 
A distributed control law based on moving path following approach is finally proposed to maintain  tracker-target's relative geometry,  which captures accurate tracking of moving vessels. 
Simulation results are shown to demonstrate the effective performance of the proposed scheme.
# Journal

# Conference
Tamba, T. A., Santjoko, I. R., Nazaruddin, Y. Y., & Nadhira, V. (2024, November). A Multi-UAV Coordination Scheme for Tracking Control of Multiple Moving Target Objects. In 2024 IEEE International Conference on Smart Mechatronics (ICSMech) (pp. 7-11). IEEE.
