#pragma once
#include <pbrt/base/bxdf.h>
#include <pbrt/util/math.h>
#include <pbrt/util/scattering.h>
#include <pbrt/util/transform.h>

namespace pbrt {
class PyramidBRDF {
  private:
	Float peakHeight;
	Float angle;
	Float angleRad;
	Float reflectance;

  public:
	PyramidBRDF() = default;
	PBRT_CPU_GPU
	PyramidBRDF(Float peakHeight, Float angle, Float reflectance)
		: peakHeight(peakHeight),
		  angle(angle),
		  angleRad(Radians(angle)),
		  reflectance(reflectance) {}
	// BxDF Interface:
	// TODO: used by ??
	PBRT_CPU_GPU BxDFFlags Flags() const {
		return BxDFFlags::SpecularReflection;
	}

	PBRT_CPU_GPU
	static constexpr const char* Name() { return "PyramidBRDF"; }

	std::string ToString() const;

	/// Give BRDF given local wo & wi.
	PBRT_CPU_GPU SampledSpectrum f(Vector3f wo, Vector3f wi,
								   TransportMode mode) const {
		return SampledSpectrum(0.0f);
	}

	PBRT_CPU_GPU int pow4(int exponent) const { return 1 << (2 * exponent); }

	PBRT_CPU_GPU void determineProbs(Vector3f inDir, Float prevProb,
									 int levelOffset, int saveOffset,
									 Float* exitProbPtr, Vector3f* outDirPtr,
									 int level, int maxLevel,
									 Vector3f (&normals)[4]) const {
		// Calculate relative probability
		Float probRelList[4];
		Float cosSum = 0;
		for (int face = 0; face < 4; face++) {
			probRelList[face] = Dot(normals[face], inDir);
			cosSum += probRelList[face];
		}
		if (cosSum != 0)
			for (int face = 0; face < 4; face++) {
				probRelList[face] /= cosSum;
				probRelList[face] *= prevProb;
			}
		// TODO: CHECK IF DIRECTIONS ARE HANDLED CORRECTLY (EG WO AND REFLECTION
		// AND SUBSEQUENT USE) Calculate exit probability
		Float probExitList[4];
		for (int face = 0; face < 4; face++) {
			Float shadow = G1(inDir, normals[face]);
			probExitList[face] = probRelList[face] * shadow;
		}

		// Calculate out dirs
		Vector3f outDirList[4];
		for (int face = 0; face < 4; face++) {
			outDirList[face] = Reflect(inDir, normals[face]);
		}

		// Save results
		for (int face = 0; face < 4; face++) {
			// *(relProbPtr + saveOffset + face) = probRelList[face];
			*(exitProbPtr + saveOffset + face) = probExitList[face];
			*(outDirPtr + saveOffset + face) = outDirList[face];
		}

		if (level == maxLevel)
			return;

		// Determine next levels
		int newLevelOffset = levelOffset + pow4(level);
		for (int face = 0; face < 4; face++) {
			int offset =
				levelOffset + face * pow4(level);  // f*4 -> 4+f*16 -> 20+f*64
			Float prob = probRelList[face] * (1.0 - probExitList[face]);
			determineProbs(outDirList[face], prob, newLevelOffset, offset,
						   exitProbPtr, outDirPtr, level + 1, maxLevel,
						   normals);
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
		// Pre definitions
		Float normalZ = std::cos(angleRad);
		Float normalXY = std::sin(angleRad);

		// Determine normals
		Vector3f normals[4] = {
			Vector3f(normalXY, 0, normalZ), Vector3f(0, normalXY, normalZ),
			Vector3f(-normalXY, 0, normalZ), Vector3f(0, -normalXY, normalZ)};

		// Determine probabilities
		// Float relativeProb[4 * 4 * 4];
		Float exitProb[4 * 4 * 4];
		Vector3f outDir[4 * 4 * 4];
		determineProbs(wo, Float(1.0), 0, 0, exitProb, outDir, 1, 3, normals);

		// Build CDF & choose
		// Float cumulativeProb[4*4*4];
		int choice;
		bool trapped = true;  // sampled not to escape
		Float prob = 0.0;
		for (int i = 0; i < 4 * 4 * 4; i++) {
			prob += exitProb[i];
			// cumulativeProb[i] = prob;
			if (uc < prob) {
				choice = i;
				trapped = false;
				break;
			}
		}
		if (trapped)	// light trapped
			return {};	// TODO: should this return valid light value of
						// intensity 0?

		Vector3f wi = outDir[choice];
		Float pdf = exitProb[choice];
		Float brdf = InvPi;	 // TODO
		// TODO: actual light value . . .

		// Float cosTheta = Dot(wo, normal);

		// return BSDFSample(SampledSpectrum(reflectance / AbsCosTheta(wi)), wi,
		// pdf, BxDFFlags::SpecularReflection);
		return BSDFSample(SampledSpectrum(brdf), wi, pdf,
						  BxDFFlags::SpecularReflection);
	}

	PBRT_CPU_GPU Float shadowing_lyanne(Vector3f v) const {
		Float thetaNormal = Radians(angle);
		Float cosThetaV = CosTheta(v);
		Float inp = std::cos(std::abs(std::acos(cosThetaV) - thetaNormal));
		Float inp2 = cosThetaV * std::cos(thetaNormal);	 // Actual Inproduct

		Float numerator = 4.0 * std::cos(thetaNormal) * cosThetaV;
		Float denominator = inp + 2 * inp2;
		if (numerator > denominator)
			return Float(1.0);
		return numerator / denominator;
	}

	PBRT_CPU_GPU Vector3f zeroAzimuth(Vector3f w) const {
		// "Rotate" wo back to wr with azimuth=phi=0
		Float x = std::sqrt(1.0 - w.z * w.z);
		return Vector3f(x, 0, w.z);
	}

	PBRT_CPU_GPU Float G1(Vector3f wo, Vector3f w) const {
		if(Dot(wo, w) < 0.0)
			return Float(0.0);
		Vector3f wr = zeroAzimuth(wo);
		return shadowing_lyanne(wr);
		// Vector3f g =
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