#pragma once
#include <pbrt/pbrt.h>
#include <pbrt/base/bssrdf.h>
#include <pbrt/base/material.h>
#include <pbrt/textures.h>

namespace pbrt {

// MaterialEvalContext Definition
struct MaterialEvalContext : public TextureEvalContext {
    // MaterialEvalContext Public Methods
    MaterialEvalContext() = default;
    PBRT_CPU_GPU
    MaterialEvalContext(const SurfaceInteraction &si)
        : TextureEvalContext(si), wo(si.wo), ns(si.shading.n), dpdus(si.shading.dpdu) {}
    std::string ToString() const;

    Vector3f wo;
    Normal3f ns;
    Vector3f dpdus;
};


// NormalBumpEvalContext Definition
struct NormalBumpEvalContext {
    // NormalBumpEvalContext Public Methods
    NormalBumpEvalContext() = default;
    PBRT_CPU_GPU
    NormalBumpEvalContext(const SurfaceInteraction &si)
        : p(si.p()),
          uv(si.uv),
          n(si.n),
          dudx(si.dudx),
          dudy(si.dudy),
          dvdx(si.dvdx),
          dvdy(si.dvdy),
          dpdx(si.dpdx),
          dpdy(si.dpdy),
          faceIndex(si.faceIndex) {
        shading.n = si.shading.n;
        shading.dpdu = si.shading.dpdu;
        shading.dpdv = si.shading.dpdv;
        shading.dndu = si.shading.dndu;
        shading.dndv = si.shading.dndv;
    }
    std::string ToString() const;

    PBRT_CPU_GPU
    operator TextureEvalContext() const {
        return TextureEvalContext(p, dpdx, dpdy, n, uv, dudx, dudy, dvdx, dvdy,
                                  faceIndex);
    }

    // NormalBumpEvalContext Public Members
    Point3f p;
    Point2f uv;
    Normal3f n;
    struct {
        Normal3f n;
        Vector3f dpdu, dpdv;
        Normal3f dndu, dndv;
    } shading;
    Float dudx = 0, dudy = 0, dvdx = 0, dvdy = 0;
    Vector3f dpdx, dpdy;
    int faceIndex = 0;
};

// Normal Mapping Function Definitions
inline PBRT_CPU_GPU void NormalMap(const Image &normalMap,
                                   const NormalBumpEvalContext &ctx, Vector3f *dpdu,
                                   Vector3f *dpdv) {
    // Get normalized normal vector from normal map
    WrapMode2D wrap(WrapMode::Repeat);
    Point2f uv(ctx.uv[0], 1 - ctx.uv[1]);
    Vector3f ns(2 * normalMap.BilerpChannel(uv, 0, wrap) - 1,
                2 * normalMap.BilerpChannel(uv, 1, wrap) - 1,
                2 * normalMap.BilerpChannel(uv, 2, wrap) - 1);
    ns = Normalize(ns);

    // Transform tangent-space normal to rendering space
    Frame frame = Frame::FromXZ(Normalize(ctx.shading.dpdu), Vector3f(ctx.shading.n));
    ns = frame.FromLocal(ns);

    // Find $\dpdu$ and $\dpdv$ that give shading normal
    Float ulen = Length(ctx.shading.dpdu), vlen = Length(ctx.shading.dpdv);
    *dpdu = Normalize(GramSchmidt(ctx.shading.dpdu, ns)) * ulen;
    *dpdv = Normalize(Cross(ns, *dpdu)) * vlen;
}

// Bump Mapping Function Definitions
template <typename TextureEvaluator>
PBRT_CPU_GPU void BumpMap(TextureEvaluator texEval, FloatTexture displacement,
                          const NormalBumpEvalContext &ctx, Vector3f *dpdu,
                          Vector3f *dpdv) {
    DCHECK(texEval.CanEvaluate({displacement}, {}));
    // Compute offset positions and evaluate displacement texture
    TextureEvalContext shiftedCtx = ctx;
    // Shift _shiftedCtx_ _du_ in the $u$ direction
    Float du = .5f * (std::abs(ctx.dudx) + std::abs(ctx.dudy));
    if (du == 0)
        du = .0005f;
    shiftedCtx.p = ctx.p + du * ctx.shading.dpdu;
    shiftedCtx.uv = ctx.uv + Vector2f(du, 0.f);

    Float uDisplace = texEval(displacement, shiftedCtx);
    // Shift _shiftedCtx_ _dv_ in the $v$ direction
    Float dv = .5f * (std::abs(ctx.dvdx) + std::abs(ctx.dvdy));
    if (dv == 0)
        dv = .0005f;
    shiftedCtx.p = ctx.p + dv * ctx.shading.dpdv;
    shiftedCtx.uv = ctx.uv + Vector2f(0.f, dv);

    Float vDisplace = texEval(displacement, shiftedCtx);
    Float displace = texEval(displacement, ctx);

    // Compute bump-mapped differential geometry
    *dpdu = ctx.shading.dpdu + (uDisplace - displace) / du * Vector3f(ctx.shading.n) +
            displace * Vector3f(ctx.shading.dndu);
    *dpdv = ctx.shading.dpdv + (vDisplace - displace) / dv * Vector3f(ctx.shading.n) +
            displace * Vector3f(ctx.shading.dndv);
}
} // namespace pbrt