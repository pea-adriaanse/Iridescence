
#include <pbrt/pbrt.h>
#include <pbrt/base/bxdf.h>
#include <string>

/// @brief Test 1
namespace pbrt {
/// @brief Test 2
class LambertianBRDF {
	LambertianBRDF() = default;
	// VxDF Interface:
	// TODO: used by ??
	PBRT_CPU_GPU inline BxDFFlags Flags() const {
		return BxDFFlags::DiffuseReflection;
	}

	std::string ToString() const;

	/// Give BRDF given local wo & wi.
	PBRT_CPU_GPU inline SampledSpectrum f(Vector3f wo, Vector3f wi,
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
	PBRT_CPU_GPU inline pstd::optional<BSDFSample> Sample_f(
		Vector3f wo, Float uc, Point2f u,
		TransportMode mode = TransportMode::Radiance,
		BxDFReflTransFlags sampleFlags = BxDFReflTransFlags::All) const {
		
	}

	// TODO
};
}
