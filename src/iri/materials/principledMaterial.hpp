#pragma once
#include <pbrt/material_util.h>

#include <iri/bxdfs/principledBRDF.hpp>

namespace pbrt {
class PrincipledMaterial {
  private:
	Image *normalMap;
	FloatTexture metallic;
	FloatTexture roughness;
	FloatTexture eta;
	FloatTexture k;

  public:
	using BxDF = PrincipledBRDF;
	using BSSRDF = void;

	static const char *Name() { return "PrincipledMaterial"; }

	static PrincipledMaterial *Create(
		const TextureParameterDictionary &parameters, Image *normalMap,
		const FileLoc *loc, Allocator alloc);

	PrincipledMaterial(Image *normalMap, FloatTexture metallic,
					   FloatTexture roughness, FloatTexture eta, FloatTexture k)
		: normalMap(normalMap),
		  metallic(metallic),
		  roughness(roughness),
		  eta(eta),
		  k(k) {}

	std::string ToString() const;

	template <typename TextureEvaluator>
	PBRT_CPU_GPU BxDF GetBxDF(TextureEvaluator texEval, MaterialEvalContext ctx,
							  SampledWavelengths &lambda) const {
		Float m = Clamp(texEval(metallic, ctx), 0.0, 1.0);
		Float r = Clamp(texEval(roughness, ctx), 0.0, 1.0);
		Float alpha = TrowbridgeReitzDistribution::RoughnessToAlpha(r);
		TrowbridgeReitzDistribution dist =
			TrowbridgeReitzDistribution(alpha, alpha);
		Float eta_ = texEval(eta, ctx);	 // TODO: does IOR have ranges?
		Float k_ = texEval(k, ctx);
		pstd::complex<Float> eta_complex = pstd::complex<Float>(eta_, k_);
		return PrincipledBRDF(dist, m, eta_complex);
	}

	template <typename TextureEvaluator>
	PBRT_CPU_GPU BSSRDF GetBSSRDF(TextureEvaluator texEval,
								  MaterialEvalContext ctx,
								  SampledWavelengths &lambda,
								  ScratchBuffer &buf) const {}

	template <typename TextureEvaluator>
	PBRT_CPU_GPU bool CanEvaluateTextures(TextureEvaluator texEval) const {
		return texEval.CanEvaluate({metallic, roughness, eta, k}, {});
	}

	PBRT_CPU_GPU const Image *GetNormalMap() const { return normalMap; }

	PBRT_CPU_GPU FloatTexture GetDisplacement() const { return nullptr; }

	PBRT_CPU_GPU bool HasSubsurfaceScattering() const { return false; }
};
}  // namespace pbrt
