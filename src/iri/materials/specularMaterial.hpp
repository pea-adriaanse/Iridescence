#pragma once
#include <pbrt/material_util.h>
#include <pbrt/util/math.h>

#include <iri/bxdfs/specularBRDF.hpp>

namespace pbrt {
class SpecularMaterial {
  private:
	Float reflectance;

  public:
	using BxDF = SpecularBRDF;
	using BSSRDF = void;

	static const char *Name() { return "SpecularMaterial"; }

	static SpecularMaterial *Create(
		const TextureParameterDictionary &parameters, Image *normalMap,
		const FileLoc *loc, Allocator alloc);

	SpecularMaterial(Image *normalMap, Float reflectance)
		: normalMap(normalMap), reflectance(reflectance) {}

	std::string ToString() const;

	template <typename TextureEvaluator>
	PBRT_CPU_GPU BxDF GetBxDF(TextureEvaluator texEval, MaterialEvalContext ctx,
							  SampledWavelengths &lambda) const {
		return SpecularBRDF(reflectance);
	}

	template <typename TextureEvaluator>
	PBRT_CPU_GPU BSSRDF GetBSSRDF(TextureEvaluator texEval,
								  MaterialEvalContext ctx,
								  SampledWavelengths &lambda,
								  ScratchBuffer &buf) const {}

	template <typename TextureEvaluator>
	PBRT_CPU_GPU bool CanEvaluateTextures(TextureEvaluator texEval) const {
		return true;
	}

	PBRT_CPU_GPU const Image *GetNormalMap() const { return normalMap; }

	PBRT_CPU_GPU FloatTexture GetDisplacement() const { return nullptr; }

	PBRT_CPU_GPU bool HasSubsurfaceScattering() const { return false; }

  private:
	Image *normalMap;
};
}  // namespace pbrt
