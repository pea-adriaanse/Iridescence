#pragma once
#include <pbrt/base/bxdf.h>
#include <pbrt/util/math.h>
#include <pbrt/util/scattering.h>
#include <pbrt/util/transform.h>
#include <iri/util.hpp>

#include <algorithm>
#include <array>
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include <cstdio>

namespace pbrt {

class PyramidBRDF {
	private:
	typedef unsigned int uint;

	Float peakHeight;
	Float angle;
	Float angleRad;
	// Float reflectance;
	uint reflectCount;
	bool shadowPaul;
	std::string setting;
	Vector3f woVector;	// when printDist is used
	Vector3f normals[4];
	std::string distOutFile;

	public:
	static constexpr uint maxLevels = 5;
	static constexpr uint maxOptionCount = pow4sum(maxLevels);

	PyramidBRDF() = default;
	PBRT_CPU_GPU
	PyramidBRDF(Float peakHeight,
				Float angle,
				uint reflectCount,
				bool shadowPaul,
				std::string setting,
				Vector3f woVector,
				std::string distOutFile)
		: peakHeight(peakHeight),
		  angle(angle),
		  angleRad(Radians(angle)),
		  reflectCount(reflectCount),
		  shadowPaul(shadowPaul),
		  setting(setting),
		  woVector(woVector),
		  distOutFile(distOutFile) {
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
	PBRT_CPU_GPU BxDFFlags Flags() const { return BxDFFlags::SpecularReflection; }

	PBRT_CPU_GPU
	static constexpr const char* Name() { return "PyramidBRDF"; }

	std::string ToString() const;

	/// Give BRDF given local wo & wi.
	PBRT_CPU_GPU SampledSpectrum f(Vector3f wo, Vector3f wi, TransportMode mode) const {
		return SampledSpectrum(0.0f);
	}

	template <uint N>
	struct ReflectDistS {
		std::array<Vector3f, N> outDirs;
		std::array<Float, N> exitBrdfs;
		std::array<Float, N> nexitBrdfs;
		std::array<Float, N> nexitProbs;
		std::array<Float, N> exitProbs;
		// std::array<Float, N> backCorrelation;
		std::array<Float, N> stolenProb;
	};
	typedef struct ReflectDistS<maxOptionCount> ReflectDist;

	static std::string reflectDistIndexToString(uint index) {
		std::string str;
		while (true) {
			const char choices[4] = {'E', 'N', 'W', 'S'};
			str.insert(0, 1, choices[index % 4]);
			if (index < 4)
				break;
			index = index / 4 - 1;
		}
		return str;
	}

	static uint reflectDistStringToIndex(std::string str) {
		assert(str.length() > 0);
		uint index = 0;
		for (int i = 0; i < str.length(); i++) {
			if (i != 0)
				index = index * 4 + 4;
			switch (str[i]) {
				case 'E':
					index += 0;
					break;
				case 'N':
					index += 1;
					break;
				case 'W':
					index += 2;
					break;
				case 'S':
					index += 3;
					break;
			}
		}
		return index;
	}

	static unsigned int getBackbounceIndex(uint parentIndex, uint ownIndex) {
		while(parentIndex > 4){
			uint face = parentIndex % 4;
			parentIndex = parentIndex / 4 - 1;
			ownIndex = ownIndex * 4 + 4 + face;
		}
		ownIndex = ownIndex * 4 + 4 + parentIndex;
		return ownIndex;
	}

	PBRT_CPU_GPU Float getCorrelation(Vector3f inDir, Vector3f outDir, bool xAxis) const {
		// if(inDir.z != 1 && outDir.z != 1) {
		// 	inDir.z = 0;
		// 	inDir = Normalize(inDir);
		// 	outDir.z = 0;
		// 	outDir = Normalize(outDir);
		// }
		// Float dot = Clamp(Dot(inDir, outDir), 0, 1);
		// Float angle = std::acos(dot);
		// return Clamp((0 - angle)/(M_PI/8), 0, 1); // linear slape

		if (outDir.z >= sin(angle) ) return 0;

		Float theta = std::acos(outDir.z);
		Float phi;
		if(xAxis)
		 	phi = std::acos(outDir.x/std::sin(theta));
		else
			phi = std::asin(outDir.x/std::sin(theta));

		Float elevationFactor;
		if (outDir.z <= 0)
			elevationFactor = 1;
		else
			elevationFactor = 1-(outDir.z/std::sin(angle));
			// elevationFactor = std::sin((theta-angle)*(M_PI / (M_PI - angle)));
		Float phiFactor = std::pow(std::cos(phi), 8);
		return elevationFactor * phiFactor;
	}

	static Float getStolenProb(Float backCorrelation, Float prob){
		// return backCorrelation * prob;
		// if(backCorrelation >= 0.98) return prob;
		constexpr float maxDelta = 0.235; // arbitrary low dot product difference.
		constexpr float maxDeltaInv = 1/maxDelta;
		Float delta = 1 - backCorrelation;
		if (delta <= maxDelta)
			// return prob * (1 - (maxDeltaInv*delta) * (maxDeltaInv*delta)); // inverted parabola
			return prob * ( 1 - delta/maxDelta ); // linear slope
		return 0;
	}

	PBRT_CPU_GPU void calcReflectDist(ReflectDist* results, const Vector3f inDir) const {
		uint parentLevelStart;
		uint parentLevelSize;

		// Zero initialize probs to permit backCorrelation stealing.
		for (uint i = 0; i < maxOptionCount; i++){
			results->stolenProb[i] = 0;
		}

		for (uint level = 0; level < reflectCount; level++) {
			const uint levelStart = pow4sum(level);
			const uint levelSize = pow4(level + 1);
			for (uint entryID = 0; entryID < levelSize; entryID += 4) {
				const uint index = levelStart + entryID;
				Vector3f childrenInDir;
				Float childrenProb;
				Float childrenBrdf;
				// Float childrenBackCorrelation;

				uint parentIndex = 0;
				uint parentFace = 0;
				// Determine input parameters
				if (level == 0) {
					childrenInDir = inDir;
					childrenProb = 1.0;
					childrenBrdf = 1.0;
					// childrenBackCorrelation = 1.0;
				} else {
					// TODO: Double check next two lines
					uint parentID = (entryID / 4);	//- 1;
					parentIndex = parentID + parentLevelStart;
					parentFace = parentID % 4;
					childrenInDir = -results->outDirs[parentIndex];
					childrenProb = results->nexitProbs[parentIndex];
					childrenBrdf = results->nexitBrdfs[parentIndex];
					// childrenBackCorrelation = results->backCorrelation[parentIndex];
				}

				// Skip zero children
				if (childrenProb == 0) {
				ZeroChildren:
					for (unsigned face = 0; face < 4; face++) {
						results->outDirs[index + face] = Vector3f(0, 0, 0);
						// results->exitProbs[index + face] = 0; Already initialized to 0
						results->nexitProbs[index + face] = 0;
						results->nexitBrdfs[index + face] = 0;
						results->exitBrdfs[index + face] = 0;
					}
					continue;
				}

				// Relative face probabilities
				Float relativeProbs[4];
				Float cosSum = 0;
				for (uint face = 0; face < 4; face++) {
					Float cos = std::max(Float(0), Dot(normals[face], childrenInDir));
					relativeProbs[face] = cos;	// * shadowing (symmetric -> normalized away)
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
					uint faceIndex = index + face;
					Vector3f outDir = Normalize(Reflect(childrenInDir, normals[face]));
					results->outDirs[faceIndex] = outDir;

					Float prob = childrenProb * relativeProbs[face];
					if(level > 0)
						prob += results->stolenProb[faceIndex];

					// Float backCorrelation = childrenBackCorrelation * Clamp(Dot(inDir, outDir), 0, 1);
					// results->backCorrelation[faceIndex] = backCorrelation;
					// if (level > 0 && level + 1 < reflectCount) {
					if (level > 0 && level + 1 < reflectCount && ((face + 2) % 4) == parentFace) {
						Float backCorrelation = getCorrelation(inDir, outDir, (parentFace%2) == 0);
						Float stolenProb = getStolenProb(backCorrelation, prob);
						prob -= stolenProb;
						// uint backBounceIndex = getBackbounceIndex(parentIndex, index + face);
						uint backBounceIndex = faceIndex * 4 + 4 + parentFace;
						// results->exitProbs[backBounceIndex] += stolenProb;
						results->stolenProb[backBounceIndex] = stolenProb;
					}

					Float shadowing = G1(outDir, normals[face]);
					Float exitProb = prob * shadowing;
					Float nexitProb = prob - exitProb;

					results->exitProbs[faceIndex] += exitProb;
					results->nexitProbs[faceIndex] = nexitProb;

					Float exitBrdf =
						childrenBrdf * shadowing;  // TODO: double check, seems wrong when
												   // considering this as statistics?
					Float nexitBrdf =
						childrenBrdf * (1 - shadowing);	 // TODO: double check, seems wrong when
														 // considering this as statistics?
					results->exitBrdfs[faceIndex] = exitBrdf;
					results->nexitBrdfs[faceIndex] = nexitBrdf;
				}
			}
			parentLevelSize = levelSize;
			parentLevelStart = levelStart;
		}
		return;
	}

	PBRT_CPU_GPU void determineProbs(Vector3f inDir,
									 Float prevProb,
									 int saveOffset,
									 Float* exitProbPtr,
									 Vector3f* outDirPtr,
									 int level,
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
			determineProbs(newInDir, prob, offset, exitProbPtr, outDirPtr, level + 1, maxLevel);
		}
	}

	/// @brief Samples the BRDF
	/// @param wo locac out direction
	/// @param uc, u uniformly sampled random values
	/// @param mode mode determining if camera or light is out direction
	/// @param sampleFlags requested sampling flags
	/// @return The local wi, the BRDF, the PDF & flags about the sample.
	PBRT_CPU_GPU pstd::optional<BSDFSample> Sample_f(
		Vector3f wo,
		Float uc,
		Point2f u,
		TransportMode mode = TransportMode::Radiance,
		BxDFReflTransFlags sampleFlags = BxDFReflTransFlags::All) const {
		// // Determine probabilities
		// constexpr int maxOptionCount = pow4sum(maxLevels);
		// Float exitProb[maxOptionCount];
		// Vector3f outDir[maxOptionCount];

		if (setting == "printDist")
			wo = woVector;

		ReflectDist reflectDist;
		calcReflectDist(&reflectDist, wo);

		unsigned int optionCount = pow4sum(reflectCount);
		// determineProbs(wo, Float(1.0), 0, exitProb, outDir, 1, reflectCount);

		// #ifdef PBRT_DEBUG_BUILD

		// FILE* file = fopen("debug.txt", "w");
		// fprintf(file, "\n%f, %f, %f\n", wo[0], wo[1], wo[2]);
		// fprintf(file, "%f, %f, %f\n", normals[0][0], normals[0][1],
		// normals[0][2]); Float exitSum = 0; for (unsigned int i = 0; i <
		// optionCount; i++) { 	Vector3f outDir = reflectDist.outDirs[i]; Float
		// exitProb = reflectDist.exitProbs[i]; 	std::string str =
		// reflectDistIndexToString(i); 	fprintf(file, "%s(%f): %f, %f,
		// %f\n", str.c_str(), exitProb, outDir[0], outDir[1], outDir[2]);
		// 	fflush(file);
		// 	exitSum += exitProb;
		// }
		// if (!(exitSum > 0 && exitSum <= 1)) {
		// 	// FILE* fErr = fopen("error.txt", "a");
		// 	fprintf(file, "Error, exit sum: %.9g\n", exitSum);
		// 	fflush(file);
		// }
		// fprintf(file, "exit sum: %f\n", exitSum);
		// fflush(file);
		// // fclose(file);
		// #endif
		// Build CDF & choose
		bool chosen = false;
		unsigned int choice = 0;
		Float probSum = 0;
		for (unsigned int i = 0; i < optionCount; i++) {
			// if(reflectDist.exitProbs[i] <= 1e-5)
			// 	continue;
			probSum += reflectDist.exitProbs[i];
			if (uc < probSum) {
				choice = i;
				chosen = true;
				break;
			}
		}

		if (setting == "printDist") {
			FILE* csv = fopen(distOutFile.c_str(), "w");
			Float exitProbSum = 0;
			for (unsigned int i = 0; i < optionCount; i++) {
				exitProbSum += reflectDist.exitProbs[i];
			}
			fprintf(csv, "index,indexStr,stolenProb,exitProb,exitBrdf\n");
			for (unsigned int i = 0; i < optionCount; i++) {
				std::string indexStr = reflectDistIndexToString(i);
				fprintf(csv, "%i,%s,%f,%f,%e\n", i, indexStr.c_str(), reflectDist.stolenProb[i], reflectDist.exitProbs[i] , reflectDist.exitBrdfs[i]);
			}
			printf("wo: %f,%f,%f\n", wo.x, wo.y, wo.z);
			fprintf(csv,
					"\nwo.x,wo.y,wo.z,uc,exitProbSum,chosen,choice\n%f,%f,%f,%"
					"f,%f,%i,%u\n\n",
					wo.x, wo.y, wo.z, uc, exitProbSum, chosen, choice);
			fflush(csv);
			fclose(csv);

			// // Write exact direction to binary file.
			// Float woDir[3] = {wo.x, wo.y, wo.z};
			// FILE* woBinary = fopen("distWo.bin",
			// 					   "wb");  // identical regardless of shadowing approach
			// fwrite(&woDir, sizeof(Float), 3, woBinary);
			// fclose(woBinary);
		}

		// if (!chosen) {
		// 	fprintf(file, "Did not choose\n");
		// } else {
		// 	std::string choiceStr = reflectDistIndexToString((unsigned int)
		// choice); 	fprintf(file, "Chose: %i (%s)\n", choice,
		// choiceStr.c_str());
		// }
		// fclose(file);
		if (!chosen)
			return {};

		// Float probSum = 0;
		// for (int i = 0; i < optionCount; i++)
		// 	probSum += (reflectDist.exitProbs[i]>0.001)? 1 : 0;
		// if (probSum == 0)
		// 	return {};

		// int choice;
		// Float prob = 0;
		// for (int i = 0; i < optionCount; i++) {
		// 	Float exitProb = ((reflectDist.exitProbs[i] > 0.001)?1:0) / probSum;
		// 	prob += exitProb;
		// 	if (uc < prob) {
		// 		choice = i;
		// 		break;
		// 	}
		// }

		Vector3f wi = reflectDist.outDirs[choice];
		// Float pdf = reflectDist.exitProbs[choice] / probSum;
		// = reflectDist.exitProbs[choice];
		Float brdf = reflectDist.exitBrdfs[choice] / AbsCosTheta(wi);
		Float pdf = 1.0;
		// Float pdf = reflectDist.exitProbs[choice];
		// Float brdf = 1;
		// Float brdf = reflectDist.brdfs[choice];

		// Float cosTheta = Dot(wo, normal);

		// return BSDFSample(SampledSpectrum(reflectance / AbsCosTheta(wi)), wi,
		// 				  pdf, BxDFFlags::SpecularReflection);
		return BSDFSample(SampledSpectrum(brdf), wi, pdf, BxDFFlags::SpecularReflection);
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
		Float x =
			std::sqrt(1.0 - w.z * w.z);	 // watch out for NaN bug (non-normal w floating error)
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

	PBRT_CPU_GPU Float PDF(Vector3f wo,
						   Vector3f wi,
						   TransportMode mode,
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