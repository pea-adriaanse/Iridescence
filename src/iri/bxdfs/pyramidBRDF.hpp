#pragma once
#include <pbrt/base/bxdf.h>
#include <pbrt/util/transform.h>
#include <pbrt/util/scattering.h>
#include <pbrt/util/math.h>

namespace pbrt {
class PyramidBRDF {
  private:
	Float peakHeight;
	Float angle;
	pstd::complex<Float> eta = pstd::complex<Float>(Float(0.0));

  public:
	PyramidBRDF() = default;
	PBRT_CPU_GPU
	PyramidBRDF(Float peakHeight, Float angle, pstd::complex<Float> eta)
		: peakHeight(peakHeight), angle(angle), eta(eta) {}
	// BxDF Interface:
	// TODO: used by ??
	PBRT_CPU_GPU BxDFFlags Flags() const {
		return BxDFFlags::SpecularReflection;
	}

	PBRT_CPU_GPU
	static constexpr const char *Name() { return "PyramidBRDF"; }

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
		// Pre definitions
		Vector3f tangent = Vector3f(
			1, 0, 0);  // TODO: configurable (should be tangential to z)
		Float angleToNormal = Radians(90.0 - 54.7);

		// Determine normals & relative areas
		Vector3f normals[4];
		Float areas[4];
		for (int i = 0; i < 4; i++) {
			normals[i] = RotateZ(90.0 * i)(tangent);
			normals[i] = std::sin(angleToNormal) * Vector3f(0, 0, 1) +
						 std::cos(angleToNormal) * normals[i];
			normals[i] = Normalize(normals[i]);

			areas[i] = Dot(normals[i], wo);
			if(areas[i] < 0) areas[i] = 0;
		}

		// Determine probabilities
		Float sum = areas[0] + areas[1] + areas[2] + areas[3];
		if (sum <= 0)
			return {};	// Invalid
		Float runningSum = 0.0;
		Float probs[4];
		for (int i = 0; i < 4; i++) {
			probs[i] = runningSum + areas[i] / sum;
			runningSum += probs[i];
		}

		// Choose normal
		Vector3f normal;
		Float pdf;
		for (int i = 0; i < 4; i++) {
			if (uc < probs[i]) {
				normal = normals[i];
				pdf = probs[i];
				break;
			}
		}

		Vector3f wi = Reflect(wo, normal);
		Float cosTheta = Dot(wo, normal);
		Float fresnel = FrComplex(cosTheta, eta);

		return BSDFSample(SampledSpectrum(fresnel / AbsCosTheta(wi)), wi, pdf,
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