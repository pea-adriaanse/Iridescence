// #pragma once
// #include <pbrt/material_util.h>

// #include <iri/bxdfs/opticalFilterBSDF.hpp>

// namespace pbrt {
// class OpticalFilterMaterial {
//   public:
// 	using BxDF = OpticalFilterBSDF;
// 	using BSSRDF = void;

// 	static const char *Name() { return "OpticalFilterMaterial"; }

// 	static OpticalFilterMaterial *Create(
// 		const TextureParameterDictionary &parameters, Image *normalMap,
// 		const FileLoc *loc, Allocator alloc);

// 	OpticalFilterMaterial(Image *normalMap, FloatTexture reflectance)
// 		: normalMap(normalMap), reflectance(reflectance) {}

// 	std::string ToString() const;

// 	template <typename TextureEvaluator>
// 	PBRT_CPU_GPU BxDF GetBxDF(TextureEvaluator texEval, MaterialEvalContext ctx,
// 							  SampledWavelengths &lambda) const {
// 		Float r = Clamp(texEval(reflectance, ctx), 0.0, 1.0);
// 		return OpticalFilterBSDF(r);
// 	}

// 	template <typename TextureEvaluator>
// 	PBRT_CPU_GPU BSSRDF GetBSSRDF(TextureEvaluator texEval,
// 								  MaterialEvalContext ctx,
// 								  SampledWavelengths &lambda,
// 								  ScratchBuffer &buf) const {}

// 	template <typename TextureEvaluator>
// 	PBRT_CPU_GPU bool CanEvaluateTextures(TextureEvaluator texEval) const {
// 		return texEval.CanEvaluate({reflectance}, {});
// 	}

// 	PBRT_CPU_GPU const Image *GetNormalMap() const { return normalMap; }

// 	PBRT_CPU_GPU FloatTexture GetDisplacement() const { return nullptr; }

// 	PBRT_CPU_GPU bool HasSubsurfaceScattering() const { return false; }

//   private:
// 	Image *normalMap;
// 	FloatTexture reflectance;
// };
// }  // namespace pbrt
