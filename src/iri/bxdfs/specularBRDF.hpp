#pragma once
#include <pbrt/base/bxdf.h>
#include <pbrt/util/math.h>
#include <pbrt/util/scattering.h>
#include <pbrt/util/transform.h>

#include <cstdio>

namespace pbrt {
class SpecularBRDF {
  private:
	Float reflectance;

  public:
	static constexpr int maxLevels = 3;
	SpecularBRDF() = default;
	PBRT_CPU_GPU
	SpecularBRDF(Float reflectance) : reflectance(reflectance) {}
	// BxDF Interface:
	// TODO: used by ??
	PBRT_CPU_GPU BxDFFlags Flags() const {
		return BxDFFlags::SpecularReflection;
	}

	PBRT_CPU_GPU
	static constexpr const char* Name() { return "SpecularBRDF"; }

	std::string ToString() const;

	/// Give BRDF given local wo & wi.
	PBRT_CPU_GPU SampledSpectrum f(Vector3f wo, Vector3f wi,
								   TransportMode mode) const {
		return SampledSpectrum(0.0f);
	}

	/// @brief Samples the BRDF
	/// @param wo locac out direction
	/// @param uc, u uniformly sampled random values
	/// @param mode mode determining if camera or light is out direction
	/// @param sampleFlags requested sampling flags
	/// @return The local wi, the BRDF, the PDF & flags about the sample.
	PBRT_CPU_GPU pstd::optional<BSDFSample> Sample_f(
		Vector3f wo, Float uc, Point2f u,
		TransportMode mode = TransportMode::Radiance,
		BxDFReflTransFlags sampleFlags = BxDFReflTransFlags::All) const {
		if (!(sampleFlags & BxDFReflTransFlags::Reflection))
			return {};

		// Determine probabilities
		Vector3f wi = Reflect(wo, Vector3f(0, 0, 1));
		Float pdf = reflectance;
		Float brdf = 1 / AbsCosTheta(wo);

		return BSDFSample(SampledSpectrum(brdf), wi, pdf,
						  BxDFFlags::SpecularReflection);
	}

	PBRT_CPU_GPU Float
	PDF(Vector3f wo, Vector3f wi, TransportMode mode,
		BxDFReflTransFlags sampleFlags = BxDFReflTransFlags::All) const {
		return 0;
	}

	// BxDF::rho already implemented
	// PBRT_CPU_GPU
	// SampledSpectrum rho(Vector3f wo, pstd::span<const Float> uc,
	// 					pstd::span<const Point2f> u2) const;
	// SampledSpectrum rho(pstd::span<const Point2f> u1,
	// 					pstd::span<const Float> uc2,
	// 					pstd::span<const Point2f> u2) const;

	PBRT_CPU_GPU void Regularize() {
		// TODO ??
	}
};
}  // namespace pbrt