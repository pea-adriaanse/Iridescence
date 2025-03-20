// #pragma once
// #include <pbrt/base/bxdf.h>
// #include <pbrt/util/error.h>
// #include <pbrt/util/math.h>
// #include <pbrt/util/print.h>
// #include <pbrt/util/scattering.h>
// #include <pbrt/util/transform.h>
// #include <iri/util.hpp>

// #include <algorithm>
// #include <array>
// #define _USE_MATH_DEFINES
// #include <math.h>
// #include <cmath>
// #include <cstdio>

// namespace pbrt {

// class OpticalFilterBSDF {
// 	public:
// 	OpticalFilterBSDF() = default;
// 	PBRT_CPU_GPU
// 	OpticalFilterBSDF(/*Float eta, TrowbridgeReitzDistribution mfDistrib*/)
// 		: /*eta(eta), mfDistrib(mfDistrib)*/ {}

// 	PBRT_CPU_GPU
// 	BxDFFlags Flags() const {
// 		return BxDFFlags::Reflection | BxDFFlags::Transmission | BxDFFlags::Specular;
// 	}

// 	PBRT_CPU_GPU
// 	pstd::optional<BSDFSample> Sample_f(
// 		Vector3f wo,
// 		Float uc,
// 		Point2f u,
// 		TransportMode mode,
// 		BxDFReflTransFlags sampleFlags = BxDFReflTransFlags::All) const;

// 	PBRT_CPU_GPU
// 	SampledSpectrum f(Vector3f wo, Vector3f wi, TransportMode mode) const;
// 	PBRT_CPU_GPU
// 	Float PDF(Vector3f wo,
// 			  Vector3f wi,
// 			  TransportMode mode,
// 			  BxDFReflTransFlags sampleFlags = BxDFReflTransFlags::All) const;

// 	PBRT_CPU_GPU
// 	static constexpr const char* Name() { return "OpticalFilterBSDF"; }

// 	std::string ToString() const;

// 	PBRT_CPU_GPU
// 	void Regularize() {}

// 	private:
// };
// }  // namespace pbrt