#pragma once
#include <pbrt/base/bxdf.h>
#include <pbrt/util/scattering.h>

namespace pbrt {
class PrincipledBRDF {
  private:
	TrowbridgeReitzDistribution dist;
	Float metallic;
	pstd::complex<Float> eta = pstd::complex<Float>(Float(0.0)); // TODO: why doesnt pstd have a default constructor??

  public:
	PrincipledBRDF() = default;
	PBRT_CPU_GPU
	PrincipledBRDF(TrowbridgeReitzDistribution dist, Float metallic,
				   pstd::complex<Float> eta)
		: dist(dist), metallic(metallic), eta(eta) {}
	// BxDF Interface:
	PBRT_CPU_GPU BxDFFlags Flags() const {
		if (dist.EffectivelySmooth())
			return BxDFFlags::SpecularReflection;
		return BxDFFlags::GlossyReflection;
	}

	PBRT_CPU_GPU
	static constexpr const char *Name() { return "PrincipledBRDF"; }

	std::string ToString() const;

	/// Give BRDF given local wo & wi.
	PBRT_CPU_GPU SampledSpectrum f(Vector3f wo, Vector3f wi,
								   TransportMode mode) const {
		if (!SameHemisphere(wo, wi) || dist.EffectivelySmooth())
			return {};

		// Calculate wm
		Vector3f wm = wo + wi;
		if (Length(wm) == 0)
			return {};
		wm = Normalize(wm);

		// Calculate metallic brdf
		Float D = dist.D(wm);
		Float F_metallic = FrComplex(AbsDot(wo, wm), eta);
		Float G = dist.G(wo, wi);
		Float cosTheta_i = AbsCosTheta(wi);
		Float cosTheta_o = AbsCosTheta(wo);
		Float div = 4 * cosTheta_i * cosTheta_o;
		// Float metallicBRDF = D*F*G/div;

		// Calculate dielectric brdf
		Float F_dielectric = FrDielectric(AbsDot(wo, wm), eta.re);

		// Return mixed result
		Float F_mix = Lerp(metallic, F_dielectric, F_metallic);
		return SampledSpectrum(D * F_mix * G / div);
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
			return {};	// None

		if (dist.EffectivelySmooth()) {
			// normal smooth sample
			Vector3f wi = Vector3f(-wo.x, -wo.y, wo.z);
			Float cosTheta_i = AbsCosTheta(wi);
			Float F_metallic = FrComplex(cosTheta_i, eta);
			Float F_dielectric = FrDielectric(cosTheta_i, eta.re);
			Float F_mix = Lerp(metallic, F_dielectric, F_metallic);
			SampledSpectrum BRDF = SampledSpectrum(F_mix / cosTheta_i);
			return BSDFSample(BRDF, wi, 1, BxDFFlags::SpecularReflection);
		}

		Vector3f wm = dist.Sample_wm(wo, u);
		Vector3f wi = Reflect(wo, wm);

		Float PDF = this->PDF(wo, wi, mode, sampleFlags);
		SampledSpectrum BRDF = this->f(wo, wi, mode);
		return BSDFSample(BRDF, wi, PDF, BxDFFlags::GlossyReflection);
	}

	PBRT_CPU_GPU Float
	PDF(Vector3f wo, Vector3f wi, TransportMode mode,
		BxDFReflTransFlags sampleFlags = BxDFReflTransFlags::All) const {
		if (!(sampleFlags & BxDFReflTransFlags::Reflection) ||
			!SameHemisphere(wo, wi) || dist.EffectivelySmooth())
			return 0;

		// Calculate wm
		Vector3f wm = wo + wi;
		if (Length(wm) == 0)
			return 0;
		wm = Normalize(wm);

		Float cosTheta_i = AbsCosTheta(wi);
		Float cosTheta_o = AbsCosTheta(wo);
		Float div = 4 * Dot(wo, wm);
		return dist.PDF(wo, wm) * div;
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