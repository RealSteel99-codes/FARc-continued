using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace FerramAerospaceResearch.FARPartGeometry
{
    [BurstCompile(FloatMode = FloatMode.Fast, FloatPrecision = FloatPrecision.Standard)]
    public struct SweepPlaneCrossSectionJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> chunkPositions;
        [ReadOnly] public NativeArray<float> chunkOccupancies;
        [ReadOnly] public float4 planeEquation; // (nx, ny, nz, d)
        [ReadOnly] public float planeStep;
        [ReadOnly] public float elementArea;
        [ReadOnly] public float elementSize;

        [WriteOnly] public NativeArray<float> outSliceAreas;
        [WriteOnly] public NativeArray<float3> outSliceCentroids;

        public void Execute(int sliceIndex)
        {
            float currentD = planeEquation.w + sliceIndex * planeStep;
            float totalArea = 0f;
            float3 centroidSum = float3.zero;

            for (int c = 0; c < chunkPositions.Length; c++)
            {
                float dist = math.dot(chunkPositions[c], planeEquation.xyz) + currentD;
                // If chunk intersects slice plane within threshold
                if (math.abs(dist) < elementSize * 4f)
                {
                    float occ = chunkOccupancies[c];
                    totalArea += occ * elementArea;
                    centroidSum += chunkPositions[c] * occ;
                }
            }

            outSliceAreas[sliceIndex] = totalArea;
            outSliceCentroids[sliceIndex] = totalArea > 1e-4f ? (centroidSum / totalArea) : float3.zero;
        }
    }
}
