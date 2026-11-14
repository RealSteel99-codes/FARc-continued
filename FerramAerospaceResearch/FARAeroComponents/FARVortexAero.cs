using System;

namespace FerramAerospaceResearch.FARAeroComponents
{
    /// <summary>
    /// Modernized vortex lift, LERX (Leading Edge Root Extension), and winglet aerodynamics model.
    /// Implements the Polhamus Leading-Edge Suction Analogy, vortex burst criteria,
    /// downstream boundary layer energization (stall delay), ground effect calculations,
    /// and Whitcomb wingtip vortex diffusion (effective AR and Oswald efficiency boost).
    /// </summary>
    public static class FARVortexAero
    {
        public enum StrakeMode
        {
            Auto = 0,
            ForceLERX = 1,
            ForceWinglet = 2,
            ForceConventional = 3,
            ForceWaverider = 4
        }

        /// <summary>
        /// Geometric heuristic check for LERX / strakes:
        /// Low aspect ratio (slender) and sharp leading edge sweep (Lambda >= 48 deg).
        /// </summary>
        public static bool IsGeometricStrake(double aspectRatio, double sweepAngleDeg)
        {
            return aspectRatio > 0.05 && aspectRatio <= 1.85 && sweepAngleDeg >= 48.0;
        }

        /// <summary>
        /// Geometric check for winglets:
        /// Significant cant/dihedral angle relative to main wing (>= 35 deg, e.g. vertical or canted),
        /// moderate aspect ratio, and height-to-span ratio typical of wingtip devices.
        /// </summary>
        public static bool IsGeometricWinglet(double dihedralAngleDeg, double wingletSpan, double mainWingSpan)
        {
            if (mainWingSpan <= 0.1)
                return false;

            double heightRatio = wingletSpan / mainWingSpan;
            double absDihedral = Math.Abs(dihedralAngleDeg);

            // Canted/vertical surface (35° to 90°) with height ratio 0.04 to 0.45
            return absDihedral >= 35.0 && heightRatio >= 0.04 && heightRatio <= 0.45;
        }

        /// <summary>
        /// Calculates the effective Aspect Ratio multiplier and Oswald efficiency boost provided by a winglet.
        /// Uses Whitcomb's semi-empirical winglet formulation:
        /// Delta AR_eff = 1.8 * (h_winglet / b_wing) * sin(cantAngle)
        /// Delta e = 0.12 * sin(cantAngle)
        /// </summary>
        public static void CalculateWingletEffect(
            double wingletSpan,
            double mainWingSpan,
            double cantAngleRad,
            out double arMultiplier,
            out double oswaldEfficiencyMultiplier)
        {
            if (mainWingSpan <= 0.1 || wingletSpan <= 0.01)
            {
                arMultiplier = 1.0;
                oswaldEfficiencyMultiplier = 1.0;
                return;
            }

            double h_over_b = Math.Min(0.40, wingletSpan / mainWingSpan);
            double sinCant = Math.Sin(Math.Abs(cantAngleRad));
            sinCant = Math.Max(0.2, sinCant); // clamp for canted winglets

            // Whitcomb's classical winglet AR increment: 1.8 * (h / b) * sin(cant)
            double deltaArFraction = 1.8 * h_over_b * sinCant;
            // Cap at 35% boost for extreme winglet heights to keep within physical limits
            deltaArFraction = Math.Min(0.35, Math.Max(0.0, deltaArFraction));
            arMultiplier = 1.0 + deltaArFraction;

            // Oswald efficiency factor increase from vortex diffusion at tip
            double deltaE = 0.14 * sinCant * (h_over_b / 0.15);
            deltaE = Math.Min(0.20, Math.Max(0.0, deltaE));
            oswaldEfficiencyMultiplier = 1.0 + deltaE;
        }

        /// <summary>
        /// Calculates the Polhamus vortex lift coefficient factor Kv based on sweep angle and Mach number.
        /// In the suction analogy, the leading-edge suction that would exist in attached flow
        /// is rotated 90 degrees to act normal to the upper surface.
        /// </summary>
        public static double CalculateKv(double sweepRad, double mach)
        {
            double cosSweep = Math.Cos(sweepRad);
            double sinSweep = Math.Sin(sweepRad);

            // Compressibility adjustment for swept leading edges
            double mTerm = 1.0 - (mach * mach * sinSweep * sinSweep);
            if (mTerm < 0.05)
                mTerm = 0.05;

            // Theoretical Polhamus suction factor: Kv = pi * cos(Lambda) / sqrt(1 - M^2 * sin^2(Lambda))
            double kv = (Math.PI * cosSweep) / Math.Sqrt(mTerm);

            // Cap at realistic physical maximum for slender wings
            return Math.Min(kv, 3.8);
        }

        /// <summary>
        /// Calculates the potential lift slope factor Kp for a low-aspect-ratio slender surface (Helmbold formula).
        /// </summary>
        public static double CalculateKp(double aspectRatio, double mach)
        {
            double ar2 = Math.Max(0.1, aspectRatio * 0.5);
            double mTerm = 1.0 - (mach * mach);
            double denom = Math.Sqrt(Math.Max(0.04, mTerm + ar2 * ar2));
            return (Math.PI * ar2) / denom;
        }

        /// <summary>
        /// Calculates the critical vortex burst (breakdown) angle of attack in radians.
        /// Higher sweep delays breakdown to higher angles of attack.
        /// Transonic shock-induced adverse pressure gradient lowers the burst angle.
        /// </summary>
        public static double CalculateBurstAoA(double sweepRad, double mach)
        {
            double sweepDeg = sweepRad * (180.0 / Math.PI);
            // Empirical baseline: for 60° sweep ~ 33° burst; for 75° sweep ~ 42° burst.
            double baseBurstDeg = 18.0 + 0.35 * sweepDeg;

            // Transonic Mach compression lowers burst threshold
            double machFactor = 1.0 / Math.Sqrt(1.0 + 0.5 * mach * mach);
            double burstDeg = baseBurstDeg * machFactor;

            return burstDeg * (Math.PI / 180.0);
        }

        /// <summary>
        /// Computes the fraction of vortex breakdown (0 = fully coherent, 1 = fully burst).
        /// </summary>
        public static double CalculateBurstFraction(double absAoA, double burstAoA)
        {
            if (absAoA <= burstAoA)
                return 0.0;

            const double burstTransitionWidthRad = 15.0 * (Math.PI / 180.0);
            double burstFrac = (absAoA - burstAoA) / burstTransitionWidthRad;
            return Math.Min(1.0, Math.Max(0.0, burstFrac));
        }

        /// <summary>
        /// Computes Polhamus non-linear aerodynamic coefficients for a strake / LERX.
        /// Normal force: Cn = Kp * sin(alpha) * cos^2(alpha) + Kv * sin^2(alpha) * cos(alpha)
        /// Lift: Cl = Cn * cos(alpha)
        /// Induced Drag: Cdi = Cn * sin(alpha)
        /// </summary>
        public static void CalculateStrakeAero(
            double aoa,
            double mach,
            double aspectRatio,
            double sweepRad,
            out double cl,
            out double cdInduced,
            out double burstFraction,
            out double acShiftFraction)
        {
            double absAoA = Math.Abs(aoa);
            double sign = Math.Sign(aoa);
            if (sign == 0) sign = 1;

            double cosAoA = Math.Cos(absAoA);
            double sinAoA = Math.Sin(absAoA);

            double kp = CalculateKp(aspectRatio, mach);
            double kv = CalculateKv(sweepRad, mach);

            double burstAoA = CalculateBurstAoA(sweepRad, mach);
            burstFraction = CalculateBurstFraction(absAoA, burstAoA);

            // As vortex breaks down, suction lift collapses
            double effectiveKv = kv * (1.0 - 0.75 * burstFraction);

            // Normal force decomposition
            double cnPotential = kp * sinAoA * cosAoA * cosAoA;
            double cnVortex = effectiveKv * sinAoA * sinAoA * cosAoA;
            double cnTotal = cnPotential + cnVortex;

            // Wind axes
            cl = sign * cnTotal * cosAoA;
            cdInduced = cnTotal * sinAoA;

            // Post-burst separated flow drag jump
            if (burstFraction > 0.0)
            {
                cdInduced += 1.3 * burstFraction * sinAoA * sinAoA;
            }

            // Aerodynamic Center shift: coherent vortex pulls AC aft;
            // burst causes loss of suction over the rear of the strake, causing pitch-up (shift forward).
            acShiftFraction = burstFraction * 0.25;
        }

        /// <summary>
        /// Calculates the stall angle delay bonus (in radians) provided to downstream main wing segments.
        /// Coherent leading-edge vortex energizes the downstream boundary layer and induces favorable axial pressure.
        /// </summary>
        public static double CalculateStallDelay(
            double strakeArea,
            double mainWingArea,
            double alignmentFactor,
            double strakeBurstFraction)
        {
            // Typical maximum stall delay for LERX-wing configurations (e.g. F/A-18, F-16) is ~+12° to +18°
            const double maxDelayRad = 15.0 * (Math.PI / 180.0);
            double areaWeight = Math.Min(1.0, (strakeArea * 2.2) / Math.Max(0.1, mainWingArea));
            double coherence = Math.Max(0.0, 1.0 - strakeBurstFraction);

            return maxDelayRad * areaWeight * Math.Max(0.0, alignmentFactor) * coherence;
        }

        /// <summary>
        /// Calculates Wieselsberger mirror-vortex ground effect parameters.
        /// </summary>
        /// <param name="altitudeAboveGround">Height of wing above surface (m)</param>
        /// <param name="wingSpan">Total wing span (m)</param>
        /// <param name="dragFactor">Induced drag reduction factor (<= 1.0)</param>
        /// <param name="liftSlopeMultiplier">Lift slope augmentation multiplier (>= 1.0)</param>
        public static void CalculateGroundEffect(
            double altitudeAboveGround,
            double wingSpan,
            out double dragFactor,
            out double liftSlopeMultiplier)
        {
            if (wingSpan <= 0.1 || altitudeAboveGround >= wingSpan * 1.5)
            {
                dragFactor = 1.0;
                liftSlopeMultiplier = 1.0;
                return;
            }

            double h_over_b = Math.Max(0.02, altitudeAboveGround / wingSpan);

            // Wieselsberger ground effect induced drag reduction formula:
            // sigma = 16 * (h/b)^2 / (1 + 16 * (h/b)^2)
            double hbSq16 = 16.0 * h_over_b * h_over_b;
            dragFactor = hbSq16 / (1.0 + hbSq16);
            dragFactor = Math.Min(1.0, Math.Max(0.15, dragFactor));

            // Lift slope augmentation from reduced downwash
            liftSlopeMultiplier = 1.0 + (1.0 - dragFactor) * 0.22;
        }

        /// <summary>
        /// Calculates aerodynamic hinge moment (torque in N*m) on a control surface.
        /// H = Ch * q * S_tab * c_tab
        /// where Ch represents the aerodynamic moment required to deflect the surface away from trail:
        /// Ch = |Ch_delta| * |delta| + |Ch_alpha| * |alpha|
        /// </summary>
        /// <param name="dynamicPressurekPa">Dynamic pressure in kPa</param>
        /// <param name="surfaceArea">Area of control surface in m^2</param>
        /// <param name="chord">Mean chord of control surface in m</param>
        /// <param name="aoaRad">Angle of attack of the surface in radians</param>
        /// <param name="deflectionRad">Surface deflection in radians</param>
        /// <param name="mach">Flight Mach number</param>
        /// <returns>Aerodynamic hinge moment restoring torque in N*m</returns>
        public static double CalculateHingeMoment(
            double dynamicPressurekPa,
            double surfaceArea,
            double chord,
            double aoaRad,
            double deflectionRad,
            double mach)
        {
            // Dynamic pressure in Pascals (N/m^2)
            double q_Pa = dynamicPressurekPa * 1000.0;
            if (q_Pa <= 1.0 || surfaceArea <= 0.001 || chord <= 0.01)
                return 0.0;

            // Compressibility Prandtl-Glauert factor for subsonic/transonic hinge slope
            double prandtl = mach < 0.9 ? 1.0 / Math.Sqrt(Math.Max(0.2, 1.0 - mach * mach)) : 1.35;
            if (mach > 1.2)
                prandtl = 1.0 / Math.Sqrt(Math.Max(0.2, mach * mach - 1.0));

            // Standard NACA empirical hinge moment derivative:
            // Ch_delta ~ -0.45 * prandtl, Ch_alpha ~ -0.20 * prandtl
            double ch_delta = 0.45 * prandtl;
            double ch_alpha = 0.20 * prandtl;

            // Torque required to hold deflection against dynamic air stream and angle of attack
            double Ch = (ch_delta * Math.Abs(deflectionRad)) + (ch_alpha * Math.Abs(aoaRad));

            return Ch * q_Pa * surfaceArea * chord;
        }

        /// <summary>
        /// Calculates the dynamic blowback allowable deflection limit given available actuator torque.
        /// When aero hinge moment equals actuator max torque capacity, deflection cannot exceed this limit.
        /// </summary>
        /// <param name="desiredDeflectionDeg">Pilot/SAS commanded deflection in degrees</param>
        /// <param name="dynamicPressurekPa">Dynamic pressure in kPa</param>
        /// <param name="surfaceArea">Area of control surface in m^2</param>
        /// <param name="chord">Mean chord of control surface in m</param>
        /// <param name="aoaRad">Angle of attack in radians</param>
        /// <param name="mach">Flight Mach number</param>
        /// <param name="maxTorqueNm">Actuator stall torque in N*m</param>
        /// <param name="blowbackRatio">Output fraction of commanded deflection achieved (0.0 to 1.0)</param>
        /// <returns>Allowed deflection in degrees</returns>
        public static double CalculateBlowbackDeflection(
            double desiredDeflectionDeg,
            double dynamicPressurekPa,
            double surfaceArea,
            double chord,
            double aoaRad,
            double mach,
            double maxTorqueNm,
            out double blowbackRatio)
        {
            if (Math.Abs(desiredDeflectionDeg) < 0.01 || maxTorqueNm <= 0.0)
            {
                blowbackRatio = 1.0;
                return desiredDeflectionDeg;
            }

            double desiredRad = desiredDeflectionDeg * (Math.PI / 180.0);
            double requiredTorque = CalculateHingeMoment(dynamicPressurekPa, surfaceArea, chord, aoaRad, desiredRad, mach);

            if (requiredTorque <= maxTorqueNm)
            {
                blowbackRatio = 1.0;
                return desiredDeflectionDeg;
            }

            // Actuator torque is exceeded: surface blows back until aero hinge moment matches actuator torque
            blowbackRatio = Math.Max(0.05, maxTorqueNm / requiredTorque);
            return desiredDeflectionDeg * blowbackRatio;
        }

        /// <summary>
        /// Heuristic check for waverider geometry:
        /// Swept delta / wedge geometry (Lambda >= 44 deg) and moderate aspect ratio (AR &lt;= 3.2).
        /// Captures classic deltas (MiG-21, Concorde, F-106, XB-70) and lifting bodies.
        /// </summary>
        public static bool IsGeometricWaverider(double aspectRatio, double sweepAngleDeg)
        {
            return aspectRatio > 0.05 && aspectRatio <= 3.2 && sweepAngleDeg >= 44.0;
        }

        /// <summary>
        /// Calculates supersonic/hypersonic waverider compression lift and wave drag reduction.
        /// When Mach >= 1.8 and sweep angle exceeds or matches the Mach angle (mu = asin(1/M)),
        /// the planar oblique shock attaches to the leading edge. The post-shock high pressure is
        /// trapped completely beneath the lower wedge, creating compression lift with minimal tip leakage.
        /// </summary>
        /// <param name="aoaRad">Angle of attack in radians</param>
        /// <param name="mach">Flight Mach number</param>
        /// <param name="sweepRad">Leading edge sweep angle in radians</param>
        /// <param name="clCompression">Output compression lift coefficient</param>
        /// <param name="cdWave">Output wave drag coefficient</param>
        /// <param name="shockAttached">Whether the oblique shock is attached to the leading edge</param>
        public static void CalculateWaveriderHypersonicAero(
            double aoaRad,
            double mach,
            double sweepRad,
            out double clCompression,
            out double cdWave,
            out bool shockAttached)
        {
            double absAoA = Math.Abs(aoaRad);
            double signAoA = Math.Sign(aoaRad);
            if (signAoA == 0) signAoA = 1.0;

            if (mach < 1.8 || absAoA < 1e-4)
            {
                clCompression = 0.0;
                cdWave = 0.0;
                shockAttached = false;
                return;
            }

            // Normal Mach component relative to the swept leading edge
            double normalMach = mach * Math.Cos(sweepRad);

            // Maximum wedge angle before shock detachment (theta_max approximation for gamma=1.4)
            // Generous detachment corridor: up to 35-45 degrees
            double thetaMaxRad = Math.Max(30.0 * (Math.PI / 180.0), 45.0 * (Math.PI / 180.0) * (1.0 - 1.0 / (mach * mach)));

            // Shock attachment:
            // Swept supersonic/hypersonic leading edge with AoA within theta_max
            double sinMachAngle = 1.0 / mach;
            double cosSweep = Math.Cos(sweepRad);
            shockAttached = (cosSweep >= sinMachAngle * 0.55) && (absAoA <= thetaMaxRad);

            const double gamma = 1.4;

            if (shockAttached)
            {
                // Oblique shock wave angle beta approximation using theta-beta-M relation:
                // For supersonic/hypersonic slender wedge: (beta / theta) ~ (gamma + 1)/2 + sqrt(((gamma+1)/2)^2 + (1/(M*theta))^2)
                double mTheta = mach * Math.Max(0.01, absAoA);
                double betaOverTheta = ((gamma + 1.0) / 4.0) + Math.Sqrt(Math.Pow((gamma + 1.0) / 4.0, 2) + 1.0 / Math.Max(0.01, mTheta * mTheta));
                double betaRad = Math.Min(Math.PI * 0.48, absAoA * betaOverTheta);

                // Normal Mach ahead of oblique shock
                double m1n = mach * Math.Sin(betaRad);
                m1n = Math.Max(1.0, m1n);

                // Post-shock pressure ratio (Rankine-Hugoniot):
                // p2/p1 = 1 + (2*gamma/(gamma+1)) * (M1n^2 - 1)
                double pRatioLower = 1.0 + (2.0 * gamma / (gamma + 1.0)) * (m1n * m1n - 1.0);

                // Lower surface pressure coefficient: Cp_lower = (2 / (gamma * M^2)) * (p2/p1 - 1)
                double cpLower = (2.0 / (gamma * mach * mach)) * (pRatioLower - 1.0);

                // Upper surface is in Prandtl-Meyer expansion fan (shielded shadow zone)
                // At high Mach, upper surface pressure rapidly approaches near-vacuum (Cp_upper -> -2 / (gamma * M^2))
                double cpUpper = -Math.Min(2.0 / (gamma * mach * mach), 0.5 * cpLower);

                // Net normal pressure coefficient: Delta Cp = Cp_lower - Cp_upper
                double deltaCp = cpLower - cpUpper;

                // Waverider entrapment factor: 3D tip spillage is diffused/blocked by the attached shock curtain.
                const double entrapmentFactor = 0.90;

                clCompression = deltaCp * Math.Cos(absAoA) * entrapmentFactor * signAoA;
                // Wave drag is the pressure force component aligned with flow: Cdw = Delta Cp * sin(alpha)
                double tareWaveDrag = 0.003 / Math.Sqrt(Math.Max(1.0, mach));
                cdWave = (deltaCp * Math.Sin(absAoA) * entrapmentFactor) + tareWaveDrag;
            }
            else
            {
                // Detached shock / shock spillage regime:
                double sinAoA = Math.Sin(absAoA);
                double cpNewtonian = 2.0 * sinAoA * sinAoA;

                clCompression = cpNewtonian * Math.Cos(absAoA) * signAoA;
                cdWave = cpNewtonian * sinAoA;
            }
        }
    }
}
