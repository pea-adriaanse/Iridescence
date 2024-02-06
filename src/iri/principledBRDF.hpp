#pragma once
#include <pbrt/base/bxdf.h>
#include <pbrt/util/scattering.h>

// TODO: doublecheck all Refracts && eta's
// We use refract as if given wi but using wo, without changing eta)
// The eta & directio assumptions dont seem to add up
// TLDR: Refract defines wi as a parameter,
// But pbrt (often/)always uses wo.
// Does this not invalidate the assumptions Refract makes with respect to eta vs
// 1/eta?

namespace pbrt {
class PrincipledBRDF {
   private:
	TrowbridgeReitzDistribution dist;
	Float metallic;
	pstd::complex<Float> eta = pstd::complex<Float>(
		Float(0.0));  // TODO: why doesnt pstd have a default constructor??

	PBRT_CPU_GPU Float fresnelMix(Vector3f wo, Normal3f normal, Float* Fc,
								  Float* Fd) const {
		Float cosTheta = Dot(wo, normal);
		*Fc = FrComplex(cosTheta, eta);
		*Fd = FrDielectric(cosTheta, eta.re);
		return metallic * *Fc + (1 - metallic) * *Fd;
	}

	/// return of false means total internal reflection
	PBRT_CPU_GPU bool transmit(Vector3f wo, Vector3f* wt,  // TODO: ADD ETAP
							   Vector3f n = Vector3f(0, 0, 1)) const {
		return Refract(wo, Normal3f(n), eta.re, nullptr, wt);
	}

	PBRT_CPU_GPU Float reflectProb(Vector3f wo, Normal3f normal, Float* R,
								   Float* T, Float* Fc, Float* Fd) const {
		*R = fresnelMix(wo, normal, Fc, Fd);
		*T = (1 - metallic) * (1 - *Fd);
		// Float N = metallic * (1 - *Fc);
		return *R / (*R + *T);
	}

	// Return false whether halfvector is backfacing light is grazing
	// Note return values: eta_act = 1 when reflect = true
	PBRT_CPU_GPU bool calcHalfVector(Vector3f wo, Vector3f wi, Vector3f* wm,
									 bool* reflected, Float* eta_act) const {
		*reflected = SameHemisphere(wo, wi);

		// find wm using snells law (or specular)
		*eta_act = *reflected ? 1 : eta.re;
		if (!*reflected && wo.z < 0)	 // TODO: light ? surface
			*eta_act = 1 / *eta_act;
		*wm = *eta_act * wi + wo;  // points into denser
		if (LengthSquared(*wm) == 0) return false;
		*wm = Normalize(*wm);
		if (wm->z < 0) *wm *= -1;  // points up

		if (SignBit(Dot(*wm, wo)) != SignBit(wo.z) ||
			SignBit(Dot(*wm, wi)) != SignBit(wi.z))
			return false;  // Backfacing (wm) normals
		return true;
	}

	PBRT_CPU_GPU Float calcRoughReflectPDF(Vector3f wo, Vector3f wm,
										   Float Pr) const {
		Float D = dist.PDF(wo, wm);
		Float cosTheta_o = AbsDot(wo, wm);	// with macro normal
		Float div = 4 * cosTheta_o;
		return D * Pr / div;
	}

	PBRT_CPU_GPU Float calcRoughTransmitPDF(Vector3f wo, Vector3f wi,
											Vector3f wm, Float Pt,
											Float eta_act) const {
		Float D = dist.PDF(wo, wm);
		Float cos_im = Dot(wi, wm);
		Float cos_om = Dot(wo, wm);
		Float num = D * std::abs(cos_om);
		Float div = Sqr(cos_im + cos_om / eta_act);
		return num * Pt / div;
	}

	PBRT_CPU_GPU SampledSpectrum calcRoughReflectBRDF(Vector3f wo, Vector3f wi,
													  Vector3f wm,
													  Float R) const {
		Float D = dist.D(wm);
		Float F = R;
		Float G = dist.G(wo, wi);
		Float cosTheta_o = CosTheta(wo);  // with macro normal
		Float cosTheta_i = CosTheta(wi);
		Float div = 4 * cosTheta_o * cosTheta_i;
		return SampledSpectrum((D * F * G) / div);
	}

	PBRT_CPU_GPU SampledSpectrum
	calcRoughTransmitBRDF(Vector3f wo, Vector3f wi, Vector3f wm, Float T,
						  Float eta_act, TransportMode mode) const {
		Float cosTheta_i = AbsCosTheta(wi);
		Float cosTheta_o = AbsCosTheta(wo);
		Float cos_om = Dot(wo, wm);
		Float cos_im = Dot(wi, wm);

		Float D = dist.D(wm);
		Float G = dist.G(wo, wi);
		Float num = D * T * G * std::abs(cos_im * cos_om);
		Float denom = Sqr(cos_im + cos_om / eta_act) * cosTheta_i * cosTheta_o;
		Float bsdf = num / denom;
		bool inverseTracing = mode == TransportMode::Radiance;
		if (inverseTracing) bsdf /= Sqr(eta_act);
		return SampledSpectrum(bsdf);
	}

   public:
	PrincipledBRDF() = default;
	PBRT_CPU_GPU
	PrincipledBRDF(TrowbridgeReitzDistribution dist, Float metallic,
				   pstd::complex<Float> eta)
		: dist(dist), metallic(metallic), eta(eta) {}
	// BxDF Interface:
	PBRT_CPU_GPU BxDFFlags Flags() const {
		if (dist.EffectivelySmooth()) return BxDFFlags::SpecularReflection;
		return BxDFFlags::GlossyReflection;
	}

	PBRT_CPU_GPU
	static constexpr const char* Name() { return "PrincipledBRDF"; }

	std::string ToString() const;

	/// Give BRDF given local wo & wi.
	PBRT_CPU_GPU SampledSpectrum f(Vector3f wo, Vector3f wi,
								   TransportMode mode) const {
		if (!SameHemisphere(wo, wi) || dist.EffectivelySmooth()) return {};

		Vector3f wm;
		bool reflected;
		Float eta_act;
		bool valid = calcHalfVector(wo, wi, &wm, &reflected, &eta_act);
		if (!valid) return {};

		Float R, T, Fc, Fd;
		reflectProb(wo, Normal3f(wm), &R, &T, &Fc, &Fd);

		if (reflected) {
			return calcRoughReflectBRDF(wo, wi, wm, R);
		} else {  // transmitted
			return calcRoughTransmitBRDF(wo, wi, wm, T, eta_act, mode);
		}
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
		bool reflect = sampleFlags & BxDFReflTransFlags::Reflection;
		bool transmit = sampleFlags & BxDFReflTransFlags::Transmission;
		if (!reflect & !transmit) return {};

		bool smooth = dist.EffectivelySmooth();
		Vector3f normal = smooth ? Vector3f(0, 0, 1) : dist.Sample_wm(wo, u);

		// TODO: assumes Fresnel equation equal regardles of entry/exit?
		// Calculate R & T & sample probabilities
		Float R, T, Fc, Fd;
		Float Pr = reflectProb(wo, Normal3f(normal), &R, &T, &Fc, &Fd);
		// Float sampleReflectProb = transmit * reflect * R + (!transmit);
		if (!transmit) Pr = 1;
		if (!reflect) Pr = 0;
		Float Pt = 1 - Pr;

		SampledSpectrum brdf;
		Float pdf;
		// Sample
		bool sampleReflect = (uc <= Pr);
		if (sampleReflect) {
			Vector3f wi = Reflect(wo, normal);	// Vector3f(-wo.x, -wo.y, wo.z);
			if (!SameHemisphere(wo, wi)) return {};

			if (smooth) {
				Float cosTheta = AbsCosTheta(wi);
				brdf = SampledSpectrum(R / cosTheta);
				pdf = Pr;
			} else {  // Rough
				brdf = calcRoughReflectBRDF(wo, wi, normal, R);
				pdf = calcRoughReflectPDF(wo, normal, Pr);
			}

			BxDFFlags flags = smooth ? BxDFFlags::SpecularReflection
									 : BxDFFlags::GlossyReflection;
			return BSDFSample(brdf, wi, pdf, flags);
		} else {  // transmit
			Vector3f wi;
			bool totalInternalReflection = !(this->transmit(wo, &wi, normal));
			if (totalInternalReflection) return {};

			Float eta_act = eta.re;	 // actual eta along ray
			if (wo.z < 0) eta_act = 1 / eta_act;

			if (smooth) {
				Float cosTheta = AbsCosTheta(wi);
				brdf = SampledSpectrum(T / cosTheta);
				pdf = Pt;
			} else {  // rough
				brdf = calcRoughTransmitBRDF(wo, wi, normal, T, eta_act, mode);
				pdf = calcRoughTransmitPDF(wo, wi, normal, Pt, eta_act);
			}

			BxDFFlags flags = smooth ? BxDFFlags::SpecularTransmission
									 : BxDFFlags::GlossyTransmission;
			return BSDFSample(brdf, wi, pdf, flags);
		}
	}

	// TODO: reconsider non-symmetric refraction everywhere (& eta)
	// TODO: reconsider total internal reflection & more sameHemisphere &
	// reflect/fract return bool

	PBRT_CPU_GPU Float
	PDF(Vector3f wo, Vector3f wi, TransportMode mode,
		BxDFReflTransFlags sampleFlags = BxDFReflTransFlags::All) const {
		bool reflectFlag = sampleFlags & BxDFReflTransFlags::Reflection;
		bool transmitFlag = sampleFlags & BxDFReflTransFlags::Transmission;
		if (!reflectFlag && !transmitFlag) return 0;
		if (dist.EffectivelySmooth()) return 0;
		if (wo.z == 0 || wi.z == 0) return 0;

		Vector3f wm;
		bool reflected;
		Float eta_act;
		bool valid = calcHalfVector(wo, wi, &wm, &reflected, &eta_act);
		if (!valid) return 0;

		// Sampling probabilities
		Float R, T, Fc, Fd;
		Float Pr = reflectProb(wo, Normal3f(wm), &R, &T, &Fc, &Fd);
		if (!transmitFlag) Pr = 1;
		if (!reflectFlag) Pr = 0;
		Float Pt = 1 - Pr;

		if (reflected) {  // TODO: into function
			return calcRoughReflectPDF(wo, wm, Pr);
		} else {  // transmitted
			return calcRoughTransmitPDF(wo, wi, wm, Pt, eta_act);
		}
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