using System;
using FerramAerospaceResearch.FARAeroComponents;

namespace FARVortexAeroTests
{
    class Program
    {
        static int Main(string[] args)
        {
            Console.WriteLine("==================================================");
            Console.WriteLine("Running FAR-Next Aerodynamics Test Suite");
            Console.WriteLine("==================================================");

            int failures = 0;

            void Assert(bool condition, string testName, string details = "")
            {
                if (condition)
                {
                    Console.WriteLine($"[PASS] {testName}");
                }
                else
                {
                    Console.ForegroundColor = ConsoleColor.Red;
                    Console.WriteLine($"[FAIL] {testName} - {details}");
                    Console.ResetColor();
                    failures++;
                }
            }

            // Test 1: Geometric Strake Detection
            Console.WriteLine("\n--- Testing Geometric Strake Heuristics ---");
            Assert(FARVortexAero.IsGeometricStrake(1.2, 65.0), "Low AR (1.2) + High Sweep (65°) detected as Strake");
            Assert(FARVortexAero.IsGeometricStrake(0.8, 70.0), "Very low AR (0.8) + 70° detected as Strake");
            Assert(!FARVortexAero.IsGeometricStrake(4.5, 25.0), "High AR (4.5) + Low Sweep (25°) rejected (Conventional Wing)");
            Assert(!FARVortexAero.IsGeometricStrake(3.0, 60.0), "High AR (3.0) with High Sweep rejected (Aspect ratio too high)");
            Assert(!FARVortexAero.IsGeometricStrake(1.0, 30.0), "Low AR (1.0) with low sweep (30°) rejected (Sweep too low)");

            // Test 2: Polhamus Kv Factor
            Console.WriteLine("\n--- Testing Polhamus Suction Factors ---");
            double kv60 = FARVortexAero.CalculateKv(60.0 * Math.PI / 180.0, 0.3);
            double kv75 = FARVortexAero.CalculateKv(75.0 * Math.PI / 180.0, 0.3);
            Assert(kv60 > 1.2 && kv60 < 2.5, "Kv for 60° sweep in expected range", $"Got {kv60:F3}");
            Assert(kv75 > 0.5 && kv75 < kv60, "Kv for 75° sweep is less than 60° (slender leading edge suction projection)", $"Got {kv75:F3}");

            // Test 3: Non-linear Vortex Lift Growth at Moderate-to-High Alpha
            Console.WriteLine("\n--- Testing Lift Polar Curves (Polhamus Analogy) ---");
            double sweep65Rad = 65.0 * Math.PI / 180.0;
            double arStrake = 1.2;

            FARVortexAero.CalculateStrakeAero(10.0 * Math.PI / 180.0, 0.3, arStrake, sweep65Rad,
                out double cl10, out double cd10, out double burst10, out _);
            FARVortexAero.CalculateStrakeAero(25.0 * Math.PI / 180.0, 0.3, arStrake, sweep65Rad,
                out double cl25, out double cd25, out double burst25, out _);
            FARVortexAero.CalculateStrakeAero(35.0 * Math.PI / 180.0, 0.3, arStrake, sweep65Rad,
                out double cl35, out double cd35, out double burst35, out _);

            Assert(cl25 > cl10 * 2.2, "Non-linear vortex lift superimposition at 25° AoA", $"cl10={cl10:F3}, cl25={cl25:F3}");
            Assert(cl35 > cl25, "Lift continues increasing up to 35° AoA (pre-burst)", $"cl25={cl25:F3}, cl35={cl35:F3}");
            Assert(burst10 == 0.0, "Vortex is fully coherent at 10° AoA", $"burst10={burst10}");
            Assert(burst25 == 0.0, "Vortex is fully coherent at 25° AoA", $"burst25={burst25}");

            // Test 4: Vortex Bursting at Extreme Alpha
            Console.WriteLine("\n--- Testing Vortex Breakdown & Burst Dynamics ---");
            double burstAoA = FARVortexAero.CalculateBurstAoA(sweep65Rad, 0.3);
            double burstAoADeg = burstAoA * (180.0 / Math.PI);
            Assert(burstAoADeg >= 35.0 && burstAoADeg <= 45.0, "Critical burst AoA is realistic (~40°)", $"Got {burstAoADeg:F1}°");

            FARVortexAero.CalculateStrakeAero(50.0 * Math.PI / 180.0, 0.3, arStrake, sweep65Rad,
                out double cl50, out double cd50, out double burst50, out double acShift50);

            Assert(burst50 > 0.5, "Vortex undergoes breakdown past burst angle (50° AoA)", $"burst50={burst50:F2}");
            Assert(acShift50 > 0.0, "Burst causes positive (pitch-up) AC shift due to rear suction collapse", $"acShift50={acShift50:F3}");
            Assert(cd50 > cd25 * 3.0, "Drag jumps dramatically post-burst due to separated flow", $"cd25={cd25:F3}, cd50={cd50:F3}");

            // Test 5: Boundary Layer Energization (Stall Delay on Main Wing)
            Console.WriteLine("\n--- Testing Downstream Stall Delay ---");
            double stallDelay = FARVortexAero.CalculateStallDelay(
                strakeArea: 3.5,
                mainWingArea: 15.0,
                alignmentFactor: 0.9,
                strakeBurstFraction: 0.0
            );
            double stallDelayDeg = stallDelay * (180.0 / Math.PI);
            Assert(stallDelayDeg >= 5.0 && stallDelayDeg <= 16.0, "Strake provides +5° to +16° stall delay to main wing", $"Got +{stallDelayDeg:F2}°");

            double burstStallDelay = FARVortexAero.CalculateStallDelay(
                strakeArea: 3.5,
                mainWingArea: 15.0,
                alignmentFactor: 0.9,
                strakeBurstFraction: 1.0
            );
            Assert(burstStallDelay == 0.0, "Stall delay bonus collapses when strake vortex bursts", $"Got {burstStallDelay}");

            // Test 6: Ground Effect
            Console.WriteLine("\n--- Testing Ground Effect Equations ---");
            FARVortexAero.CalculateGroundEffect(altitudeAboveGround: 2.0, wingSpan: 12.0,
                out double dragFactorNear, out double liftSlopeNear);
            FARVortexAero.CalculateGroundEffect(altitudeAboveGround: 30.0, wingSpan: 12.0,
                out double dragFactorHigh, out double liftSlopeHigh);

            Assert(dragFactorNear < 0.6, "Induced drag is slashed near ground (h/b = 0.16)", $"Got dragFactor={dragFactorNear:F3}");
            Assert(liftSlopeNear > 1.05, "Lift slope is boosted near ground", $"Got liftSlopeMultiplier={liftSlopeNear:F3}");
            Assert(Math.Abs(dragFactorHigh - 1.0) < 1e-4, "Induced drag returns to 100% in free flight (h/b > 1.5)", $"Got {dragFactorHigh}");

            // Test 7: Winglet Whitcomb Effect & Induced Drag Reduction
            Console.WriteLine("\n--- Testing Winglet Induced Drag Reduction (Whitcomb Model) ---");
            double mainSpan = 14.0;
            double wingletHeight = 1.4; // h/b = 0.10
            double cant90Rad = 90.0 * Math.PI / 180.0;
            double cant60Rad = 60.0 * Math.PI / 180.0;

            Assert(FARVortexAero.IsGeometricWinglet(85.0, wingletHeight, mainSpan), "Vertical wingtip fin detected as Winglet");
            Assert(FARVortexAero.IsGeometricWinglet(55.0, wingletHeight, mainSpan), "Canted 55° winglet detected as Winglet");
            Assert(!FARVortexAero.IsGeometricWinglet(10.0, wingletHeight, mainSpan), "Flat extension (10° dihedral) rejected (Conventional wing)");
            Assert(!FARVortexAero.IsGeometricWinglet(85.0, 10.0, mainSpan), "Massive fin (h/b > 0.45) rejected as Winglet");

            FARVortexAero.CalculateWingletEffect(wingletHeight, mainSpan, cant90Rad, out double arMult90, out double eMult90);
            FARVortexAero.CalculateWingletEffect(wingletHeight, mainSpan, cant60Rad, out double arMult60, out double eMult60);

            Assert(arMult90 >= 1.15 && arMult90 <= 1.25, "90° Winglet provides +15% to +25% effective AR boost", $"Got arMult={arMult90:F3}");
            Assert(eMult90 >= 1.05 && eMult90 <= 1.15, "90° Winglet provides +5% to +15% Oswald efficiency boost", $"Got eMult={eMult90:F3}");
            Assert(arMult90 > arMult60, "Vertical winglet provides higher vortex diffusion than lower cant", $"arMult90={arMult90:F3}, arMult60={arMult60:F3}");

            // Induced drag comparison: Cdi = Cl^2 / (pi * AR * e)
            double baseAr = 7.0;
            double baseE = 0.80;
            double baseDenom = baseAr * baseE;
            double wingletDenom = (baseAr * arMult90) * (baseE * eMult90);
            double cdiReductionFraction = 1.0 - (baseDenom / wingletDenom);
            Assert(cdiReductionFraction >= 0.18 && cdiReductionFraction <= 0.30, "Winglet cuts induced drag by 18% to 30%", $"Got -{cdiReductionFraction * 100:F1}%");

            Console.WriteLine("\n--- Testing Aeroelastic Control Blowback & Hinge Moments ---");
            // Low dynamic pressure (slow flight, 50 m/s, q ~ 1.5 kPa): full deflection easily achieved
            double lowQ = 1.5;
            double surfArea = 0.5; // 0.5 m^2
            double surfChord = 0.3; // 0.3 m
            double lowMach = 0.15;
            double stallTorque = 1500.0; // 1500 N*m actuator capacity
            double defl1 = FARVortexAero.CalculateBlowbackDeflection(20.0, lowQ, surfArea, surfChord, 0.05, lowMach, stallTorque, out double blowback1);
            Assert(blowback1 > 0.999 && Math.Abs(defl1 - 20.0) < 0.01, "Low dynamic pressure allows full 20° deflection", $"Got {defl1:F1}°, ratio={blowback1:F2}");

            // High dynamic pressure dive / supersonic dash (q = 80 kPa, Mach 1.5): massive hinge moment causes blowback
            double highQ = 80.0;
            double highMach = 1.5;
            double highStallTorque = 600.0; // 600 N*m realistic actuator capacity for light surface
            double defl2 = FARVortexAero.CalculateBlowbackDeflection(20.0, highQ, surfArea, surfChord, 0.05, highMach, highStallTorque, out double blowback2);
            Assert(blowback2 < 0.5, "High-q supersonic flight blows back deflection to < 50%", $"Got {defl2:F1}°, ratio={blowback2:F2}");
            Assert(defl2 < 10.0 && defl2 > 1.0, "Effective deflection is realistically constrained", $"Got {defl2:F1}°");

            double hingeNm = FARVortexAero.CalculateHingeMoment(highQ, surfArea, surfChord, 0.05, 20.0 * Math.PI / 180.0, highMach);
            Assert(hingeNm > highStallTorque, "High-q commanded hinge moment exceeds available actuator torque", $"Hinge torque = {hingeNm:F0} N*m vs Stall = {highStallTorque} N*m");

            Console.WriteLine("\n--- Testing Waverider Hypersonic Compression Lift ---");
            // Test geometry heuristics
            Assert(FARVortexAero.IsGeometricWaverider(1.2, 70.0), "High sweep (70°) + low AR (1.2) detected as Waverider", "Expected true");
            Assert(!FARVortexAero.IsGeometricWaverider(4.0, 30.0), "Low sweep (30°) + high AR (4.0) rejected as Waverider", "Expected false");

            // Test Mach 6 hypersonic compression lift
            double testMach = 6.0;
            double testSweepRad = 68.0 * (Math.PI / 180.0);
            double testAoA = 6.0 * (Math.PI / 180.0); // 6 deg trim AoA
            FARVortexAero.CalculateWaveriderHypersonicAero(testAoA, testMach, testSweepRad, out double clWave, out double cdWave, out bool shockAttached);

            Assert(shockAttached, "Shock attaches along sharp swept leading edge at Mach 6", "Expected shockAttached = true");
            Assert(clWave > 0.05, "Compression lift generated under lower surface", $"Got Cl = {clWave:F4}");
            double ldRatio = clWave / Math.Max(1e-4, cdWave);
            Assert(ldRatio >= 4.0 && ldRatio <= 8.5, "Waverider compression lift yields superior L/D (4.0 to 8.5)", $"Got L/D = {ldRatio:F2}");

            // Test extreme high AoA shock detachment
            double highAoA = 50.0 * (Math.PI / 180.0); // 50 deg AoA exceeds theta_max
            FARVortexAero.CalculateWaveriderHypersonicAero(highAoA, testMach, testSweepRad, out double clDetached, out double cdDetached, out bool shockAttachedHigh);
            Assert(!shockAttachedHigh, "Shock detaches when AoA exceeds maximum wedge shock angle", "Expected shockAttached = false");
            double ldDetached = clDetached / Math.Max(1e-4, cdDetached);
            Assert(ldDetached < 1.5, "Detached shock degrades into high-drag blunt flow (L/D < 1.5)", $"Got L/D = {ldDetached:F2}");

            Console.WriteLine("\n==================================================");
            if (failures == 0)
            {
                Console.ForegroundColor = ConsoleColor.Green;
                Console.WriteLine("ALL TESTS PASSED!");
                Console.ResetColor();
                return 0;
            }
            else
            {
                Console.ForegroundColor = ConsoleColor.Red;
                Console.WriteLine($"{failures} TESTS FAILED!");
                Console.ResetColor();
                return 1;
            }
        }
    }
}
