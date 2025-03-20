#pragma once
#include <pbrt/material_util.h>
#include <pbrt/util/math.h>

#include <iri/bxdfs/pyramidBRDF.hpp>

namespace pbrt {
class PyramidMaterial {
	private:
	Image* normalMap;
	Float peakHeight;
	Float angle;
	int reflectCount;
	bool shadowPaul;
	bool rebounce;
	Float reflectance;
	std::string setting;
	Vector3f woVector;
	std::string distOutFile;

	public:
	using BxDF = PyramidBRDF;
	using BSSRDF = void;

	static const char* Name() { return "PyramidMaterial"; }

	static PyramidMaterial* Create(const TextureParameterDictionary& parameters,
								   Image* normalMap,
								   const FileLoc* loc,
								   Allocator alloc);

	PyramidMaterial(Image* normalMap,
					Float peakHeight,
					Float angle,
					int reflectCount,
					bool shadowPaul,
					bool rebounce,
					Float reflectance,
					std::string setting,
					Vector3f woVector,
					std::string distOutFile)
		: normalMap(normalMap),
		  peakHeight(peakHeight),
		  angle(angle),
		  reflectCount(reflectCount),
		  shadowPaul(shadowPaul),
		  rebounce(rebounce),
		  reflectance(reflectance),
		  setting(setting),
		  woVector(woVector),
		  distOutFile(distOutFile) {}

	std::string ToString() const;

	static Float OpticalFilter(Float lambda, Float angle);

	template <typename TextureEvaluator>
	PBRT_CPU_GPU BxDF GetBxDF(TextureEvaluator texEval,
							  MaterialEvalContext ctx,
							  SampledWavelengths& lambda) const {  // Lambda!!
		// Float h = texEval(peakHeight, ctx);
		// Float a = Clamp(texEval(angle, ctx), 0.0, 90.0);
		// Float r = Clamp(texEval(reflectance, ctx), 0.0, 1.0);
		// int reflectCount = texEval()
		std::array<Float, NSpectrumSamples> lambdas;
		for (int i = 0; i < NSpectrumSamples; i++)
			lambdas[i] = lambda[i];

		return PyramidBRDF(peakHeight, angle, reflectCount, shadowPaul, rebounce, reflectance,
						   lambdas, &OpticalFilter, setting, woVector, distOutFile);
	}

	template <typename TextureEvaluator>
	PBRT_CPU_GPU BSSRDF GetBSSRDF(TextureEvaluator texEval,
								  MaterialEvalContext ctx,
								  SampledWavelengths& lambda,
								  ScratchBuffer& buf) const {}

	template <typename TextureEvaluator>
	PBRT_CPU_GPU bool CanEvaluateTextures(TextureEvaluator texEval) const {
		// return texEval.CanEvaluate({}, {});
		return true;
	}

	PBRT_CPU_GPU const Image* GetNormalMap() const { return normalMap; }

	PBRT_CPU_GPU FloatTexture GetDisplacement() const { return nullptr; }

	PBRT_CPU_GPU bool HasSubsurfaceScattering() const { return false; }
};
}  // namespace pbrt
