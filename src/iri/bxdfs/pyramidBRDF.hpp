#pragma once
#include <pbrt/base/bxdf.h>
#include <pbrt/util/math.h>
#include <pbrt/util/scattering.h>
#include <pbrt/util/transform.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

namespace pbrt {
// 1, 4, 16, 64
PBRT_CPU_GPU static constexpr unsigned int pow4(unsigned int exponent) {
	return 1 << (2 * exponent);
}

// 0, 4, 20, 84
PBRT_CPU_GPU static constexpr unsigned int pow4sum(unsigned int exponent) {
	return (4 * (pow4(exponent) - 1)) / 3;	// Modified geometric series
}

class PyramidBRDF {
  private:
	typedef unsigned int uint;

	Float peakHeight;
	Float angle;
	Float angleRad;
	// Float reflectance;
	int reflectCount;
	bool shadowPaul;
	Vector3f normals[4];

  public:
	static constexpr uint maxLevels = 5;
	static constexpr uint maxOptionCount = pow4sum(maxLevels);

	PyramidBRDF() = default;
	PBRT_CPU_GPU
	PyramidBRDF(Float peakHeight, Float angle, int reflectCount,
				bool shadowPaul)
		: peakHeight(peakHeight),
		  angle(angle),
		  angleRad(Radians(angle)),
		  reflectCount(reflectCount),
		  shadowPaul(shadowPaul) {
		// Determine normals
		Float normalZ = std::cos(angleRad);
		Float normalXY = std::sin(angleRad);

		this->normals[0] = Vector3f(normalXY, 0, normalZ);
		this->normals[1] = Vector3f(0, normalXY, normalZ);
		this->normals[2] = Vector3f(-normalXY, 0, normalZ);
		this->normals[3] = Vector3f(0, -normalXY, normalZ);
		// 0.816137612
		// 0.00000000
		// 0.577857614
	}
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

	template <uint N>
	struct ReflectDistS {
		std::array<Vector3f, N> outDirs;
		std::array<Float, N> nexitProbs;
		std::array<Float, N> exitProbs;
	};
	typedef struct ReflectDistS<maxOptionCount> ReflectDist;

	PBRT_CPU_GPU void calcReflectDist(ReflectDist* results,
									  const Vector3f inDir) const {
		uint parentLevelStart;
		uint parentLevelSize;

		for (uint level = 0; level < reflectCount; level++) {
			const uint levelStart = pow4sum(level);
			const uint levelSize = pow4(level + 1);
			for (uint entryID = 0; entryID < levelSize; entryID += 4) {
				const uint index = levelStart + entryID;
				Vector3f childrenInDir;
				Float childrenProb;

				// Determine input parameters
				if (level == 0) {
					childrenInDir = inDir;
					childrenProb = 1.0;
				} else {
					uint parentID = (entryID / 4) - 1;
					uint parent = parentID + parentLevelStart;
					childrenInDir = -results->outDirs[parent];
					childrenProb = results->nexitProbs[parent];
				}

				// Skip zero children
				if (childrenProb == 0) {
				ZeroChildren:
					for (unsigned face = 0; face < 4; face++) {
						results->outDirs[index + face] = Vector3f(0, 0, 0);
						results->exitProbs[index + face] = 0;
						results->nexitProbs[index + face] = 0;
					}
					continue;
				}

				// Relative face probabilities
				Float relativeProbs[4];
				Float cosSum = 0;
				for (uint face = 0; face < 4; face++) {
					Float cos = std::max(Float(0), Dot(normals[face], inDir));
					relativeProbs[face] =
						cos;  // * shadowing (symmetric -> normalized away)
					cosSum += cos;
				}

				// Normalize
				if (cosSum > 0)
					for (unsigned face = 0; face < 4; face++) {
						relativeProbs[face] /= cosSum;
					}
				else
					goto ZeroChildren;

				// Results
				for (uint face = 0; face < 4; face++) {
					Vector3f outDir = Reflect(inDir, normals[face]);
					results->outDirs[index + face] = outDir;

					Float prob = childrenProb * relativeProbs[face];
					Float shadowing = G1(outDir, normals[face]);
					Float exitProb = prob * shadowing;
					Float nexitProb = prob - exitProb;
					results->exitProbs[index + face] = exitProb;
					results->nexitProbs[index + face] = nexitProb;
				}
			}
			parentLevelSize = levelSize;
			parentLevelStart = levelStart;
		}
		return;
	}

	PBRT_CPU_GPU void determineProbs(Vector3f inDir, Float prevProb,
									 int saveOffset, Float* exitProbPtr,
									 Vector3f* outDirPtr, int level,
									 int maxLevel) const {
		Float probList[4];
		Float probSum = 0;

		// Calculate probabilities
		for (int face = 0; face < 4; face++) {
			Float cos = Dot(normals[face], inDir);
			cos = cos > 0 ? cos : 0;
			probSum += cos;
			probList[face] = cos;
		}
		if (probSum != 0)
			for (int face = 0; face < 4; face++) {
				probList[face] /= probSum;	// normalize
				probList[face] *= prevProb;
			}

		// Calculate out dirs
		Vector3f outDirList[4];
		for (int face = 0; face < 4; face++)
			outDirList[face] = Reflect(inDir, normals[face]);

		// Calculate exit probability & shadowing
		Float probExitList[4];
		Float shadows[4];
		for (int face = 0; face < 4; face++) {
			Float shadow = G1(outDirList[face], normals[face]);
			shadows[face] = shadow;
			probExitList[face] = shadow * probList[face];
		}

		// Save results
		for (int face = 0; face < 4; face++) {
			*(exitProbPtr + saveOffset + face) = probExitList[face];
			*(outDirPtr + saveOffset + face) = outDirList[face];
		}

		if (level == maxLevel)
			return;

		// Recurse to next bounces
		for (int face = 0; face < 4; face++) {
			int offset = (saveOffset + face + 1) * 4;
			Float prob = probList[face] - probExitList[face];
			Vector3f newInDir = -outDirList[face];	// flip direction
			determineProbs(newInDir, prob, offset, exitProbPtr, outDirPtr,
						   level + 1, maxLevel);
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
		// // Determine probabilities
		// constexpr int maxOptionCount = pow4sum(maxLevels);
		// Float exitProb[maxOptionCount];
		// Vector3f outDir[maxOptionCount];

		ReflectDist reflectDist;
		calcReflectDist(&reflectDist, wo);

		int optionCount = pow4sum(reflectCount);
		// determineProbs(wo, Float(1.0), 0, exitProb, outDir, 1, reflectCount);

#ifdef PBRT_DEBUG_BUILD
		FILE* file = fopen("debug.txt", "w");
		fprintf(file, "%f, %f, %f\n", wo[0], wo[1], wo[2]);
		fprintf(file, "%f, %f, %f\n", normals[0][0], normals[0][1],
				normals[0][2]);
		Float exitSum = 0;
		for (int i = 0; i < optionCount; i++) {
			fprintf(file, "%f, %f, %f, %f\n", outDir[i][0], outDir[i][1],
					outDir[i][2], exitProb[i]);
			fflush(file);
			exitSum += exitProb[i];
		}
		if (!(exitSum > 0 && exitSum <= 1)) {
			FILE* fErr = fopen("error.txt", "a");
			fprintf(fErr, "%.9g\n", exitSum);
			fclose(fErr);
		}
		fprintf(file, "%f\n", exitSum);
		fflush(file);
		fclose(file);
#endif

		// Build CDF & choose
		// Float cumulativeProb[4*4*4];
		int choice;
		bool trapped = true;  // sampled not to escape
		Float prob = 0.0;
		for (int i = 0; i < optionCount; i++) {
			prob += reflectDist.exitProbs[i];
			// cumulativeProb[i] = prob;
			if (uc < prob) {
				choice = i;
				trapped = false;
				break;
			}
		}

		if (trapped)  // light trapped
			return BSDFSample(
				SampledSpectrum(0), wo, 0,
				BxDFFlags::SpecularReflection);	 // TODO: should this return
												 // valid light value of
												 // intensity 0?

		Vector3f wi = reflectDist.outDirs[choice];
		Float pdf = reflectDist.exitProbs[choice];
		Float brdf = pdf / AbsCosTheta(wi);

		// Float cosTheta = Dot(wo, normal);

		// return BSDFSample(SampledSpectrum(reflectance / AbsCosTheta(wi)), wi,
		// 				  pdf, BxDFFlags::SpecularReflection);
		return BSDFSample(SampledSpectrum(brdf), wi, pdf,
						  BxDFFlags::SpecularReflection);
	}

	PBRT_CPU_GPU Float G1(Vector3f wo, Vector3f n) const {
		Float shadowing;
		if (shadowPaul)
			shadowing = shadowing_paul(wo);
		else
			shadowing = shadowing_lyanne(wo, n);
		return Clamp(shadowing, 0.0, 1.0);
	}

	PBRT_CPU_GPU Float shadowing_paul(Vector3f wo) const {
		Float C_cam = 0;
		for (int i = 0; i < 4; i++) {
			Float C_i = Dot(wo, this->normals[i]);
			if (C_i > 0)
				C_cam += C_i;
		}
		if (C_cam == 0)
			return 0.0;
		return 4 * std::cos(angleRad) * CosTheta(wo) / C_cam;
	}

	PBRT_CPU_GPU static Vector3f zeroAzimuth(Vector3f w) {
		// "Rotate" wo back to wr with azimuth=phi=0
		Float x = std::sqrt(1.0 - w.z * w.z);
		return Vector3f(x, 0, w.z);
	}

	PBRT_CPU_GPU Float shadowing_lyanne(Vector3f o, Vector3f v) const {
		if (Dot(o, v) < 0.0)
			return Float(0.0);
		Vector3f r = zeroAzimuth(o);

		Float rg = CosTheta(r);
		if (rg <= 0.0)
			return 0.0;
		Float D = 1.0 / (4.0 * std::cos(angleRad));
		Float rfe = (r.z * normals[0].z) + (r.x * normals[0].x);
		Float rfo = r.z * normals[0].z;

		Float numerator = rg;
		Float denominator = D * (rfe + 2 * rfo);
		Float shadowing = numerator / denominator;
		return shadowing;
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