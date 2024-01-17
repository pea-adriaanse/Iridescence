#include <pbrt/pbrt.h>

namespace pbrt {
class LambertianBRDF {
  public:
	LambertianBRDF() = default;
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
		return SampledSpectrum(InvPi);	// Assumed R = 1 (full reflection)
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
			pstd::optional<BSDFSample>();  // None
		}
		if (mode == TransportMode::Importance) {
			assert(0);
			// TODO
		}
		BSDFSample sample;
		sample.wi = SampleCosineHemisphere(u);
		sample.f = f(wo, sample.wi, mode);
		sample.pdf = PDF(wo, sample.wi, mode, sampleFlags);
		// sample.eta = ?
		// sample.flags = ?
		// sample.pdfIsProportional = ?
		return sample;
	}

	PBRT_CPU_GPU Float
	PDF(Vector3f wo, Vector3f wi, TransportMode mode,
		BxDFReflTransFlags sampleFlags = BxDFReflTransFlags::All) const {
		float cosine = CosTheta(wo);
		return CosineHemispherePDF(cosine);
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