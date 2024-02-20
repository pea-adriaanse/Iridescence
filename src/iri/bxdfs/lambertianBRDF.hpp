#pragma once
#include <pbrt/base/bxdf.h>

namespace pbrt {
class LambertianBRDF {
  private:
	Float reflectance;

  public:
	LambertianBRDF() = default;
	PBRT_CPU_GPU
	LambertianBRDF(Float reflectance) : reflectance(reflectance) {}
	// BxDF Interface:
	// TODO: used by ??
	PBRT_CPU_GPU BxDFFlags Flags() const {
		return BxDFFlags::DiffuseReflection;
	}

	PBRT_CPU_GPU
	static constexpr const char *Name() { return "LambertianBRDF"; }

	std::string ToString() const;

	/// Give BRDF given local wo & wi.
	PBRT_CPU_GPU SampledSpectrum f(Vector3f wo, Vector3f wi,
								   TransportMode mode) const {
		if (!SameHemisphere(wo, wi)) {
			return SampledSpectrum(0.0f);
		}
		return SampledSpectrum(reflectance * InvPi);
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
		if (!(sampleFlags & BxDFReflTransFlags::Reflection)) {
			return {};	// None
		}
		Vector3f wi = SampleCosineHemisphere(u);
		if (wo.z < 0)
			wi.z *= -1;
		// SampledSpectrum f_ = f(wo, wi, mode);
		// Float pdf = PDF(wo, wi, mode, sampleFlags);
		// BxDFFlags flags = BxDFFlags::DiffuseReflection;
		// return BSDFSample(f_, wi, pdf, flags);
		Float pdf = CosineHemispherePDF(AbsCosTheta(wi));

        return BSDFSample(SampledSpectrum(reflectance * InvPi), wi, pdf, BxDFFlags::DiffuseReflection);
	}

	PBRT_CPU_GPU Float
	PDF(Vector3f wo, Vector3f wi, TransportMode mode,
		BxDFReflTransFlags sampleFlags = BxDFReflTransFlags::All) const {
		if (!(sampleFlags & BxDFReflTransFlags::Reflection) ||
			!SameHemisphere(wo, wi))
			return 0;
		// Float cosine = AbsCosTheta(wi);
		// return CosineHemispherePDF(cosine);
		return CosineHemispherePDF(AbsCosTheta(wi));
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