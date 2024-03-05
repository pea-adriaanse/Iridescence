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
	FloatTexture reflectance;

   public:
	using BxDF = PyramidBRDF;
	using BSSRDF = void;

	static const char *Name() { return "PyramidMaterial"; }

	static PyramidMaterial *Create(const TextureParameterDictionary &parameters,
								   Image *normalMap, const FileLoc *loc,
								   Allocator alloc);

	PyramidMaterial(Image *normalMap, FloatTexture peakHeight,
					FloatTexture angle, FloatTexture reflectance)
		: normalMap(normalMap),
		  peakHeight(peakHeight),
		  angle(angle),
		  reflectance(reflectance) {}

	std::string ToString() const;

	template <typename TextureEvaluator>
	PBRT_CPU_GPU BxDF GetBxDF(TextureEvaluator texEval, MaterialEvalContext ctx,
							  SampledWavelengths &lambda) const {
		Float h = texEval(peakHeight, ctx);
		Float a = Clamp(texEval(angle, ctx), 0.0, 90.0);
		Float r = Clamp(texEval(reflectance, ctx), 0.0, 1.0);
		return PyramidBRDF(h, a, r);
	}

	template <typename TextureEvaluator>
	PBRT_CPU_GPU BSSRDF GetBSSRDF(TextureEvaluator texEval,
								  MaterialEvalContext ctx,
								  SampledWavelengths &lambda,
								  ScratchBuffer &buf) const {}

	template <typename TextureEvaluator>
	PBRT_CPU_GPU bool CanEvaluateTextures(TextureEvaluator texEval) const {
		return texEval.CanEvaluate({peakHeight, angle, reflectance}, {});
	}

	PBRT_CPU_GPU const Image *GetNormalMap() const { return normalMap; }

	PBRT_CPU_GPU FloatTexture GetDisplacement() const { return nullptr; }

	PBRT_CPU_GPU bool HasSubsurfaceScattering() const { return false; }
};
}  // namespace pbrt
