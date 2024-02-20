#pragma once
#include <pbrt/material_util.h>
#include <pbrt/util/math.h>

#include <iri/bxdfs/pyramidBRDF.hpp>

namespace pbrt {
class PyramidMaterial {
  private:
	Image *normalMap;
	FloatTexture peakHeight;
	FloatTexture angle;
	FloatTexture eta;
	FloatTexture k;

  public:
	using BxDF = PyramidBRDF;
	using BSSRDF = void;

	static const char *Name() { return "PyramidMaterial"; }

	static PyramidMaterial *Create(const TextureParameterDictionary &parameters,
								   Image *normalMap, const FileLoc *loc,
								   Allocator alloc);

	PyramidMaterial(Image *normalMap, FloatTexture peakHeight,
					FloatTexture angle, FloatTexture eta, FloatTexture k)
		: normalMap(normalMap),
		  peakHeight(peakHeight),
		  angle(angle),
		  eta(eta),
		  k(k) {}

	std::string ToString() const;

	template <typename TextureEvaluator>
	PBRT_CPU_GPU BxDF GetBxDF(TextureEvaluator texEval, MaterialEvalContext ctx,
							  SampledWavelengths &lambda) const {
		Float h = texEval(peakHeight, ctx);
		Float a = Clamp(texEval(angle, ctx), 0.0, PiOver2);
		Float eta_ = texEval(eta, ctx);	 // TODO: does IOR have ranges?
		Float k_ = texEval(k, ctx);
		pstd::complex<Float> eta_complex = pstd::complex<Float>(eta_, k_);
		return PyramidBRDF(h, a, eta_complex);
	}

	template <typename TextureEvaluator>
	PBRT_CPU_GPU BSSRDF GetBSSRDF(TextureEvaluator texEval,
								  MaterialEvalContext ctx,
								  SampledWavelengths &lambda,
								  ScratchBuffer &buf) const {}

	template <typename TextureEvaluator>
	PBRT_CPU_GPU bool CanEvaluateTextures(TextureEvaluator texEval) const {
		return texEval.CanEvaluate({peakHeight, angle, eta, k}, {});
	}

	PBRT_CPU_GPU const Image *GetNormalMap() const { return normalMap; }

	PBRT_CPU_GPU FloatTexture GetDisplacement() const { return nullptr; }

	PBRT_CPU_GPU bool HasSubsurfaceScattering() const { return false; }
};
}  // namespace pbrt
