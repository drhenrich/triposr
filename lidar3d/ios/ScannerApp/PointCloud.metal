// Punktwolken-Shader.
//
// Die Punkte liegen als float3 im Gerätespeicher (16 Byte Stride, passend zu
// SIMD3<Float> auf der Swift-Seite). Eingefärbt wird über die Höhe - das ist
// der grüne Verlauf aus der Vorlage.

#include <metal_stdlib>
using namespace metal;

struct Uniforms {
    float4x4 mvp;
    float pointSize;
    float zMin;
    float zMax;
};

struct VertexOut {
    float4 position [[position]];
    float pointSize [[point_size]];
    half3 color;
};

vertex VertexOut pointVertex(uint vid [[vertex_id]],
                             const device float3 *points [[buffer(0)]],
                             constant Uniforms &u [[buffer(1)]])
{
    float3 p = points[vid];

    VertexOut out;
    out.position = u.mvp * float4(p, 1.0);
    out.pointSize = u.pointSize;

    float span = max(u.zMax - u.zMin, 1e-3);
    float t = clamp((p.z - u.zMin) / span, 0.0, 1.0);
    out.color = half3(half(0.10 * t), half(0.35 + 0.65 * t), half(0.22 + 0.28 * t));
    return out;
}

fragment half4 pointFragment(VertexOut in [[stage_in]],
                             float2 coord [[point_coord]])
{
    // Runde statt quadratischer Punkte - bei dichten Wolken deutlich lesbarer.
    if (length(coord - float2(0.5)) > 0.5) {
        discard_fragment();
    }
    return half4(in.color, 1.0h);
}
