/*
  Copyright© (c) 2014-2017 Youen Toupin, (aka neuoy).
  Copyright© (c) 2014-2018 A.Korsunsky, (aka fat-lobyte).
  Copyright© (c) 2017-2020 S.Gray, (aka PiezPiedPy).

  This file is part of Trajectories.
  Trajectories is available under the terms of GPL-3.0-or-later.
  See the LICENSE.md file for more details.

  Trajectories is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Trajectories is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

  You should have received a copy of the GNU General Public License
  along with Trajectories.  If not, see <http://www.gnu.org/licenses/>.
*/

using System;
using System.Collections.Generic;
using System.Linq;
using KSP.Localization;
using UnityEngine;

namespace Trajectories
{
    /// <summary>
    /// Handles trajectory prediction, performing a lightweight physical simulation to
    /// predict a vessels trajectory in space and atmosphere.
    /// </summary>
    public class Trajectory
    {
        internal class VesselState
        {
            internal CelestialBody ReferenceBody { get; set; }

            // universal time
            internal double Time { get; set; }

            // position in world frame relatively to the reference body
            internal Vector3d Position { get; set; }

            // velocity in world frame relatively to the reference body
            internal Vector3d Velocity { get; set; }

            // tells whether the patch starting from this state is superimposed on a stock KSP patch, or null if
            // something makes it diverge (atmospheric entry for example)
            internal Orbit StockPatch { get; set; }

            internal VesselState()
            {
            }

            internal VesselState(Vessel vessel)
            {
                ReferenceBody = vessel.orbit.referenceBody;
                Time = Planetarium.GetUniversalTime();
                Position = vessel.GetWorldPos3D() - ReferenceBody.position;
                Velocity = vessel.obt_velocity;
                StockPatch = vessel.orbit;
            }
        }

        internal struct Point
        {
            internal Vector3d pos;
            internal Vector3d aerodynamicForce;
            internal Vector3d orbitalVelocity;

            /// <summary>
            /// Ground altitude above (or under) sea level, in meters.
            /// </summary>
            internal float groundAltitude;

            /// <summary>
            /// Universal time
            /// </summary>
            internal double time;
        }

        internal class Patch
        {
            internal VesselState StartingState { get; set; }

            internal double EndTime { get; set; }

            internal bool IsAtmospheric { get; set; }

            // // position array in body space (world frame centered on the body) ; only used when isAtmospheric is true
            internal Point[] AtmosphericTrajectory { get; set; }

            // only used when isAtmospheric is false
            internal Orbit SpaceOrbit { get; set; }

            internal Vector3d? ImpactPosition { get; set; }

            internal Vector3d? RawImpactPosition { get; set; }

            internal Vector3d? ImpactVelocity { get; set; }
        }

        internal const double INTEGRATOR_MIN = 0.1d; // RK4 Integrator minimum step size
        internal const double INTEGRATOR_MAX = 5.0d; // RK4 Integrator maximum step size

        private const double MAX_INCREMENT_TIME = 2.0d; // in ms

        private VesselAerodynamicModel aerodynamicModel_;

        internal string AerodynamicModelName => aerodynamicModel_ == null ? Localizer.Format("#autoLOC_Trajectories_NotLoaded") : aerodynamicModel_.AerodynamicModelName;

        private List<Patch> patchesBackBuffer_ = new List<Patch>();

        internal List<Patch> Patches { get; private set; } = new List<Patch>();

        private float maxAccelBackBuffer_;

        internal float MaxAccel { get; private set; }

        internal int ErrorCount { get; private set; }

        private IEnumerator<bool> partialComputation_;

        private double increment_time;

        ///<summary> Trajectory calculation time of last frame in ms </summary>
        internal double ComputationTime { get; private set; }

        private double frame_time;

        ///<summary> Time of game frame in ms </summary>
        internal double GameFrameTime { get; private set; }

        /// <summary> The vessel that trajectories is attached to </summary>
        internal Vessel AttachedVessel { get; }

        internal TargetProfile TargetProfile { get; }

        internal DescentProfile DescentProfile { get; }

        internal FlightOverlay FlightOverlay { get; }

        internal MapOverlay MapOverlay { get; }

        internal NavBallOverlay NavBallOverlay { get; }

        /// <returns> True if trajectories is attached to a vessel </returns>
        internal bool IsVesselAttached => AttachedVessel != null;

        /// <returns> True if trajectories is attached to a vessel and that the vessel also has parts </returns>
        internal bool VesselHasParts => IsVesselAttached && AttachedVessel.Parts.Count != 0;

        internal Trajectory(Vessel vessel)
        {
            AttachedVessel = vessel;
            TargetProfile = new TargetProfile(this);
            DescentProfile = new DescentProfile(this);
            FlightOverlay = new FlightOverlay(this);
            MapOverlay = new MapOverlay(this);
            NavBallOverlay = new NavBallOverlay(this);
            Util.DebugLog("Constructing");
        }

        internal void Destroy()
        {
            Util.DebugLog("");
            DescentProfile.Destroy();
            FlightOverlay.Destroy();
            NavBallOverlay.Destroy();
            MapOverlay.Destroy();
        }

        internal void Update()
        {
            //Profiler.Start("Trajectory.Update");

            // compute game frame time
            GameFrameTime = GameFrameTime * 0.9d + Util.ElapsedMilliseconds(frame_time) * 0.1d;
            frame_time = Util.Clocks;

            // should the trajectory be calculated?
            if (Settings.DisplayTrajectories || Settings.AlwaysUpdate || TargetProfile.WorldPosition.HasValue)
                ComputeTrajectory();

            // compute computation time
            ComputationTime = ComputationTime * 0.9d + increment_time * 0.1d;

            //Profiler.Stop("Trajectory.Update");
            FlightOverlay.Update();
            MapOverlay.Update();
            NavBallOverlay.Update();
        }

        internal void InvalidateAerodynamicModel() => aerodynamicModel_.Invalidate();

        internal void ComputeTrajectory()
        {
            if (!VesselHasParts)// || AttachedVessel.LandedOrSplashed)
            {
                increment_time = 0d;
                Patches.Clear();
                return;
            }

            try
            {
                // start of trajectory calculation in current frame
                increment_time = Util.Clocks;

                // create or update aerodynamic model
                if (aerodynamicModel_ == null || !aerodynamicModel_.IsValidFor(AttachedVessel.mainBody))
                    aerodynamicModel_ = AerodynamicModelFactory.GetModel(this, AttachedVessel.mainBody);
                else
                    aerodynamicModel_.UpdateVesselMass();

                // if there is no ongoing partial computation, start a new one
                if (partialComputation_ == null)
                {
                    //total_time = Util.Clocks;

                    // restart the public buffers
                    patchesBackBuffer_.Clear();
                    maxAccelBackBuffer_ = 0;

                    // Create enumerator for Trajectory increment calculator
                    partialComputation_ = ComputeTrajectoryIncrement().GetEnumerator();
                }

                // we are finished when there are no more partial computations to be done
                bool finished = !partialComputation_.MoveNext();

                // when calculation is finished,
                if (finished)
                {
                    // swap the buffers for the patches and the maximum acceleration,
                    // "publishing" the results
                    List<Patch> tmp = Patches;
                    Patches = patchesBackBuffer_;
                    patchesBackBuffer_ = tmp;

                    MaxAccel = maxAccelBackBuffer_;

                    // Reset partial computation
                    partialComputation_.Dispose();
                    partialComputation_ = null;

                    // how long did the whole calculation take?
                    //total_time = Util.ElapsedMilliseconds(total_time);
                }

                // how long did the calculation in this frame take?
                increment_time = Util.ElapsedMilliseconds(increment_time);
            }
            catch (Exception)
            {
                ++ErrorCount;
                throw;
            }
        }

        private IEnumerable<bool> ComputeTrajectoryIncrement()
        {
            // create new VesselState from vessel
            VesselState state = new VesselState(AttachedVessel);

            // iterate over patches until MaxPatchCount is reached
            for (int patchIdx = 0; patchIdx < Settings.MaxPatchCount; ++patchIdx)
            {
                // stop if we don't have a vessel state
                if (state == null)
                    break;

                // If we spent more time in this calculation than allowed, pause until the next frame
                if (Util.ElapsedMilliseconds(increment_time) > MAX_INCREMENT_TIME)
                    yield return false;

                // if we have a patched conics solver, check for maneuver nodes
                if (null != AttachedVessel.patchedConicSolver)
                {
                    // search through maneuver nodes of the vessel
                    List<ManeuverNode> maneuverNodes = AttachedVessel.patchedConicSolver.maneuverNodes;
                    foreach (ManeuverNode node in maneuverNodes)
                    {
                        // if the maneuver node time corresponds to the end time of the last patch
                        if (node.UT == state.Time)
                        {
                            // add the velocity change of the burn to the velocity of the last patch
                            state.Velocity += node.GetBurnVector(CreateOrbitFromState(state));
                            break;
                        }
                    }
                }

                // Add one patch, then pause execution after every patch
                foreach (bool unused in AddPatch(state))
                    yield return false;

                state = AddPatch_outState;
            }
        }

        /// <summary>
        /// relativePosition is in world frame, but relative to the body (i.e. inertial body space)
        /// returns the altitude above sea level (can be negative for bodies without ocean)
        /// </summary>
        internal static double GetGroundAltitude(CelestialBody body, Vector3d relativePosition)
        {
            if (body.pqsController == null)
                return 0;

            double lat = body.GetLatitude(relativePosition + body.position) / 180.0 * Math.PI;
            double lon = body.GetLongitude(relativePosition + body.position) / 180.0 * Math.PI;
            Vector3d rad = new Vector3d(Math.Cos(lat) * Math.Cos(lon), Math.Sin(lat), Math.Cos(lat) * Math.Sin(lon));
            double elevation = body.pqsController.GetSurfaceHeight(rad) - body.Radius;
            if (body.ocean)
                elevation = Math.Max(elevation, 0.0);

            return elevation;
        }

        internal static double RealMaxAtmosphereAltitude(CelestialBody body)
        {
            if (!body.atmosphere)
                return 0;
            // Change for 1.0 refer to atmosphereDepth
            return body.atmosphereDepth;
        }

        private static Orbit CreateOrbitFromState(VesselState state)
        {
            Orbit orbit = new Orbit();
            orbit.UpdateFromStateVectors(state.Position.SwapYZ(), state.Velocity.SwapYZ(), state.ReferenceBody, state.Time);
            PatchedConics.SolverParameters pars = new PatchedConics.SolverParameters
            {
                FollowManeuvers = false
            };
            PatchedConics.CalculatePatch(orbit, new Orbit(), state.Time, pars, null);
            return orbit;
        }

        private double FindOrbitBodyIntersection(Orbit orbit, double startTime, double endTime, double bodyAltitude)
        {
            // binary search of entry time in atmosphere
            // I guess an analytic solution could be found, but I'm too lazy to search it

            double from = startTime;
            double to = endTime;

            int loopCount = 0;
            while (to - from > 0.1)
            {
                ++loopCount;
                if (loopCount > 1000)
                {
                    Util.LogWarning("Infinite loop? Trajectory.AddPatch or atmosphere limit search");
                    ++ErrorCount;
                    break;
                }

                double middle = (from + to) * 0.5;
                if (orbit.getRelativePositionAtUT(middle).magnitude < bodyAltitude)
                {
                    to = middle;
                }
                else
                {
                    from = middle;
                }
            }

            return to;
        }

        private VesselState AddPatch_outState;

        private struct SimulationState
        {
            internal Vector3d position;
            internal Vector3d velocity;
        }

        /// <summary>
        /// Integration step function for Runge-Kutta 4 integration
        /// </summary>
        /// <param name="state">Position+Velocity of the current time step</param>
        /// <param name="accelerationFunc">Functor that returns the acceleration for a given Position+Velocity</param>
        /// <param name="dt">Time step interval</param>
        /// <param name="accel">Stores the value of the accelerationFunc at the current time step</param>
        /// <returns></returns>
        private static SimulationState RK4Step(SimulationState state, Func<Vector3d, Vector3d, Vector3d> accelerationFunc, double dt, out Vector3d accel)
        {
            Vector3d p1 = state.position;
            Vector3d v1 = state.velocity;
            accel = accelerationFunc(p1, v1);

            Vector3d p2 = state.position + 0.5 * v1 * dt;
            Vector3d v2 = state.velocity + 0.5 * accel * dt;
            Vector3d a2 = accelerationFunc(p2, v2);

            Vector3d p3 = state.position + 0.5 * v2 * dt;
            Vector3d v3 = state.velocity + 0.5 * a2 * dt;
            Vector3d a3 = accelerationFunc(p3, v3);

            Vector3d p4 = state.position + v3 * dt;
            Vector3d v4 = state.velocity + a3 * dt;
            Vector3d a4 = accelerationFunc(p4, v4);

            state.position = state.position + (dt / 6.0) * (v1 + 2.0 * v2 + 2.0 * v3 + v4);
            state.velocity = state.velocity + (dt / 6.0) * (accel + 2.0 * a2 + 2.0 * a3 + a4);

            return state;
        }

        private IEnumerable<bool> AddPatch(VesselState startingState)
        {
            if (null == AttachedVessel.patchedConicSolver)
                AttachedVessel.AttachPatchedConicsSolver();

            CelestialBody body = startingState.ReferenceBody;

            Patch patch = new Patch
            {
                StartingState = startingState,
                IsAtmospheric = false,
                SpaceOrbit = startingState.StockPatch ?? CreateOrbitFromState(startingState)
            };
            patch.EndTime = patch.StartingState.Time + patch.SpaceOrbit.period;

            // the flight plan does not always contain the first patches (before the first maneuver node),
            // so we populate it with the current orbit and associated encounters etc.
            List<Orbit> flightPlan = new List<Orbit>();
            for (Orbit orbit = AttachedVessel.orbit; orbit != null && orbit.activePatch; orbit = orbit.nextPatch)
            {
                if (AttachedVessel.patchedConicSolver.flightPlan.Contains(orbit))
                    break;
                flightPlan.Add(orbit);
            }

            foreach (Orbit orbit in AttachedVessel.patchedConicSolver.flightPlan)
            {
                flightPlan.Add(orbit);
            }


            Orbit nextPatch = null;
            if (startingState.StockPatch == null)
            {
                nextPatch = patch.SpaceOrbit.nextPatch;
            }
            else
            {
                int planIdx = flightPlan.IndexOf(startingState.StockPatch);
                if (planIdx >= 0 && planIdx < flightPlan.Count - 1)
                {
                    nextPatch = flightPlan[planIdx + 1];
                }
            }

            if (nextPatch != null)
            {
                patch.EndTime = nextPatch.StartUT;
            }

            double maxAtmosphereAltitude = RealMaxAtmosphereAltitude(body);
            if (!body.atmosphere)
            {
                maxAtmosphereAltitude = body.pqsController.mapMaxHeight;
            }

            double minAltitude = patch.SpaceOrbit.PeA;
            if (patch.SpaceOrbit.timeToPe < 0 || patch.EndTime < startingState.Time + patch.SpaceOrbit.timeToPe)
            {
                minAltitude = Math.Min(
                    patch.SpaceOrbit.getRelativePositionAtUT(patch.EndTime).magnitude,
                    patch.SpaceOrbit.getRelativePositionAtUT(patch.StartingState.Time + 1.0).magnitude
                ) - body.Radius;
            }

            if (minAltitude < maxAtmosphereAltitude)
            {
                double entryTime;
                if (startingState.Position.magnitude <= body.Radius + maxAtmosphereAltitude)
                {
                    // whole orbit is inside the atmosphere
                    entryTime = startingState.Time;
                }
                else
                {
                    entryTime = FindOrbitBodyIntersection(
                        patch.SpaceOrbit,
                        startingState.Time, startingState.Time + patch.SpaceOrbit.timeToPe,
                        body.Radius + maxAtmosphereAltitude);
                }

                if (entryTime > startingState.Time + 0.1 || !body.atmosphere)
                {
                    if (body.atmosphere)
                    {
                        // add the space patch before atmospheric entry

                        patch.EndTime = entryTime;
                        patchesBackBuffer_.Add(patch);
                        AddPatch_outState = new VesselState
                        {
                            Position = patch.SpaceOrbit.getRelativePositionAtUT(entryTime).SwapYZ(),
                            ReferenceBody = body,
                            Time = entryTime,
                            Velocity = patch.SpaceOrbit.getOrbitalVelocityAtUT(entryTime).SwapYZ()
                        };
                    }
                    else
                    {
                        // the body has no atmosphere, so what we actually computed is the entry
                        // inside the "ground sphere" (defined by the maximal ground altitude)
                        // now we iterate until the inner ground sphere (minimal altitude), and
                        // check if we hit the ground along the way
                        double groundRangeExit = FindOrbitBodyIntersection(
                            patch.SpaceOrbit,
                            startingState.Time, startingState.Time + patch.SpaceOrbit.timeToPe,
                            body.Radius - maxAtmosphereAltitude);

                        if (groundRangeExit <= entryTime)
                            groundRangeExit = startingState.Time + patch.SpaceOrbit.timeToPe;

                        double iterationSize = (groundRangeExit - entryTime) / 100.0;
                        double t;
                        bool groundImpact = false;
                        for (t = entryTime; t < groundRangeExit; t += iterationSize)
                        {
                            Vector3d pos = patch.SpaceOrbit.getRelativePositionAtUT(t);
                            double groundAltitude = GetGroundAltitude(body, CalculateRotatedPosition(body, pos.SwapYZ(), t))
                                                    + body.Radius;
                            if (pos.magnitude < groundAltitude)
                            {
                                t -= iterationSize;
                                groundImpact = true;
                                break;
                            }
                        }

                        if (groundImpact)
                        {
                            patch.EndTime = t;
                            patch.RawImpactPosition = patch.SpaceOrbit.getRelativePositionAtUT(t).SwapYZ();
                            patch.ImpactPosition = CalculateRotatedPosition(body, patch.RawImpactPosition.Value, t);
                            patch.ImpactVelocity = patch.SpaceOrbit.getOrbitalVelocityAtUT(t).SwapYZ();
                            patchesBackBuffer_.Add(patch);
                            AddPatch_outState = null;
                        }
                        else
                        {
                            // no impact, just add the space orbit
                            patchesBackBuffer_.Add(patch);
                            if (nextPatch != null)
                            {
                                AddPatch_outState = new VesselState
                                {
                                    Position = patch.SpaceOrbit.getRelativePositionAtUT(patch.EndTime).SwapYZ(),
                                    Velocity = patch.SpaceOrbit.getOrbitalVelocityAtUT(patch.EndTime).SwapYZ(),
                                    ReferenceBody = nextPatch.referenceBody,
                                    Time = patch.EndTime,
                                    StockPatch = nextPatch
                                };
                            }
                            else
                            {
                                AddPatch_outState = null;
                            }
                        }
                    }
                }
                else
                {
                    if (patch.StartingState.ReferenceBody != AttachedVessel.mainBody)
                    {
                        // currently, we can't handle predictions for another body, so we stop
                        AddPatch_outState = null;
                        yield break;
                    }

                    // simulate atmospheric flight (drag and lift), until impact or atmosphere exit
                    // (typically for an aerobraking maneuver) assuming a constant angle of attack
                    patch.IsAtmospheric = true;
                    patch.StartingState.StockPatch = null;

                    // lower dt would be more accurate, but a trade-off has to be found between performances and accuracy
                    double dt = Settings.IntegrationStepSize;

                    // some shallow entries can result in very long flight. For performances reasons,
                    // we limit the prediction duration
                    int maxIterations = (int) (60.0 * 60.0 / dt);

                    int chunkSize = 128;

                    // time between two consecutive stored positions (more intermediate positions are computed for better accuracy),
                    // also used for ground collision checks
                    double trajectoryInterval = 10.0;

                    List<Point[]> buffer = new List<Point[]>
                    {
                        new Point[chunkSize]
                    };
                    int nextPosIdx = 0;

                    SimulationState state;
                    state.position = patch.SpaceOrbit.getRelativePositionAtUT(entryTime).SwapYZ();
                    state.velocity = patch.SpaceOrbit.getOrbitalVelocityAtUT(entryTime).SwapYZ();

                    // Initialize a patch with zero acceleration
                    Vector3d currentAccel = new Vector3d(0.0, 0.0, 0.0);


                    double currentTime = entryTime;
                    double lastPositionStoredUT = 0;
                    Vector3d lastPositionStored = new Vector3d();
                    bool hitGround = false;
                    int iteration = 0;
                    int incrementIterations = 0;
                    int minIterationsPerIncrement = maxIterations / Settings.MaxFramesPerPatch;

                    #region Acceleration Functor

                    // function that calculates the acceleration under current parameters
                    Vector3d AccelerationFunc(Vector3d position, Vector3d velocity)
                    {
                        Profiler.Start("accelerationFunc inside");

                        // gravity acceleration
                        double R_ = position.magnitude;
                        Vector3d accel_g = position * (-body.gravParameter / (R_ * R_ * R_));

                        // aero force
                        Vector3d vel_air = velocity - body.getRFrmVel(body.position + position);

                        double aoa = DescentProfile.GetAngleOfAttack(body, position, vel_air) ?? 0d;

                        Profiler.Start("GetForces");
                        Vector3d force_aero = aerodynamicModel_.GetForces(body, position, vel_air, aoa);
                        Profiler.Stop("GetForces");

                        Vector3d accel = accel_g + force_aero / aerodynamicModel_.Mass;

                        accel += API.HandleAccel(AttachedVessel, body, position, velocity, aoa);

                        Profiler.Stop("accelerationFunc inside");
                        return accel;
                    }

                    #endregion


                    #region Integration Loop

                    while (true)
                    {
                        ++iteration;
                        ++incrementIterations;

                        if (incrementIterations > minIterationsPerIncrement && Util.ElapsedMilliseconds(increment_time) > MAX_INCREMENT_TIME)
                        {
                            yield return false;
                            incrementIterations = 0;
                        }


                        double R = state.position.magnitude;
                        double altitude = R - body.Radius;
                        double atmosphereCoeff = altitude / maxAtmosphereAltitude;
                        if (hitGround
                            || atmosphereCoeff <= 0.0 || atmosphereCoeff >= 1.0
                            || iteration == maxIterations || currentTime > patch.EndTime)
                        {
                            //Util.PostSingleScreenMessage("atmo force", "Atmospheric accumulated force: " + accumulatedForces.ToString("0.00"));

                            if (hitGround || atmosphereCoeff <= 0.0)
                            {
                                patch.RawImpactPosition = state.position;
                                patch.ImpactPosition = CalculateRotatedPosition(body, patch.RawImpactPosition.Value, currentTime);
                                patch.ImpactVelocity = state.velocity;
                            }

                            patch.EndTime = Math.Min(currentTime, patch.EndTime);

                            int totalCount = (buffer.Count - 1) * chunkSize + nextPosIdx;
                            patch.AtmosphericTrajectory = new Point[totalCount];
                            int outIdx = 0;
                            foreach (Point[] chunk in buffer)
                            {
                                foreach (Point p in chunk)
                                {
                                    if (outIdx == totalCount)
                                        break;
                                    patch.AtmosphericTrajectory[outIdx++] = p;
                                }
                            }

                            if (iteration == maxIterations)
                            {
                                ScreenMessages.PostScreenMessage("WARNING: trajectory prediction stopped, too many iterations");
                                patchesBackBuffer_.Add(patch);
                                AddPatch_outState = null;
                                yield break;
                            }

                            if (atmosphereCoeff <= 0.0 || hitGround)
                            {
                                patchesBackBuffer_.Add(patch);
                                AddPatch_outState = null;
                                yield break;
                            }

                            patchesBackBuffer_.Add(patch);
                            AddPatch_outState = new VesselState
                            {
                                Position = state.position,
                                Velocity = state.velocity,
                                ReferenceBody = body,
                                Time = patch.EndTime
                            };
                            yield break;
                        }

                        Vector3d lastAccel = currentAccel;
                        SimulationState lastState = state;

                        Profiler.Start("IntegrationStep");

                        // Verlet integration (more precise than using the velocity)
                        // state = VerletStep(state, accelerationFunc, dt);
                        state = RK4Step(state, AccelerationFunc, dt, out currentAccel);

                        currentTime += dt;

                        // KSP presumably uses Euler integration for position updates. Since RK4 is actually more precise than that,
                        // we try to reintroduce an approximation of the error.

                        // The local truncation error for euler integration is:
                        // LTE = 1/2 * h^2 * y''(t)
                        // https://en.wikipedia.org/wiki/Euler_method#Local_truncation_error
                        //
                        // For us,
                        // h is the time step of the outer simulation (KSP), which is the physics time step
                        // y''(t) is the difference of the velocity/acceleration divided by the physics time step
                        state.position += 0.5 * TimeWarp.fixedDeltaTime * currentAccel * dt;
                        state.velocity += 0.5 * TimeWarp.fixedDeltaTime * (currentAccel - lastAccel);

                        Profiler.Stop("IntegrationStep");

                        // calculate gravity and aerodynamic force
                        Vector3d gravityAccel = lastState.position * (-body.gravParameter / (R * R * R));
                        Vector3d aerodynamicForce = (currentAccel - gravityAccel) / aerodynamicModel_.Mass;

                        // acceleration in the vessel reference frame is acceleration - gravityAccel
                        maxAccelBackBuffer_ = Math.Max(
                            (float) (aerodynamicForce.magnitude / aerodynamicModel_.Mass),
                            maxAccelBackBuffer_);

                        #region Impact Calculation

                        Profiler.Start("AddPatch#impact");

                        double interval = altitude < 10000.0 ? trajectoryInterval * 0.1 : trajectoryInterval;
                        if (currentTime >= lastPositionStoredUT + interval)
                        {
                            double groundAltitude = GetGroundAltitude(body, CalculateRotatedPosition(body, state.position, currentTime));
                            if (lastPositionStoredUT > 0)
                            {
                                // check terrain collision, to detect impact on mountains etc.
                                Vector3 rayOrigin = lastPositionStored;
                                Vector3 rayEnd = state.position;
                                double absGroundAltitude = groundAltitude + body.Radius;
                                if (absGroundAltitude > rayEnd.magnitude)
                                {
                                    hitGround = true;
                                    float coeff = Math.Max(0.01f, (float) ((absGroundAltitude - rayOrigin.magnitude)
                                                                           / (rayEnd.magnitude - rayOrigin.magnitude)));
                                    state.position = rayEnd * coeff + rayOrigin * (1.0f - coeff);
                                    currentTime = currentTime * coeff + lastPositionStoredUT * (1.0f - coeff);
                                }
                            }

                            lastPositionStoredUT = currentTime;
                            if (nextPosIdx == chunkSize)
                            {
                                buffer.Add(new Point[chunkSize]);
                                nextPosIdx = 0;
                            }

                            Vector3d nextPos = state.position;
                            if (Settings.BodyFixedMode)
                            {
                                nextPos = CalculateRotatedPosition(body, nextPos, currentTime);
                            }

                            buffer.Last()[nextPosIdx].aerodynamicForce = aerodynamicForce;
                            buffer.Last()[nextPosIdx].orbitalVelocity = state.velocity;
                            buffer.Last()[nextPosIdx].groundAltitude = (float) groundAltitude;
                            buffer.Last()[nextPosIdx].time = currentTime;
                            buffer.Last()[nextPosIdx++].pos = nextPos;
                            lastPositionStored = state.position;
                        }

                        Profiler.Stop("AddPatch#impact");

                        #endregion
                    }

                    #endregion
                }
            }
            else
            {
                // no atmospheric entry, just add the space orbit
                patchesBackBuffer_.Add(patch);
                if (nextPatch != null)
                {
                    AddPatch_outState = new VesselState
                    {
                        Position = patch.SpaceOrbit.getRelativePositionAtUT(patch.EndTime).SwapYZ(),
                        Velocity = patch.SpaceOrbit.getOrbitalVelocityAtUT(patch.EndTime).SwapYZ(),
                        ReferenceBody = nextPatch.referenceBody,
                        Time = patch.EndTime,
                        StockPatch = nextPatch
                    };
                }
                else
                {
                    AddPatch_outState = null;
                }
            }
        }

        internal static Vector3d CalculateRotatedPosition(CelestialBody body, Vector3d relativePosition, double time)
        {
            float angle = (float) (-(time - Planetarium.GetUniversalTime()) * body.angularVelocity.magnitude / Math.PI * 180.0);
            Quaternion bodyRotation = Quaternion.AngleAxis(angle, body.angularVelocity.normalized);
            return bodyRotation * relativePosition;
        }

        internal static Vector3d GetWorldPositionAtUT(Orbit orbit, double ut)
        {
            Vector3d worldPos = orbit.getRelativePositionAtUT(ut).SwapYZ();
            if (orbit.referenceBody != FlightGlobals.Bodies[0])
                worldPos += GetWorldPositionAtUT(orbit.referenceBody.orbit, ut);
            return worldPos;
        }
    }
}
