// pbrt is Copyright(c) 1998-2020 Matt Pharr, Wenzel Jakob, and Greg Humphreys.
// The pbrt source code is licensed under the Apache License, Version 2.0.
// SPDX: Apache-2.0

#ifndef PBRT_OPTIONS_H
#define PBRT_OPTIONS_H

#include <pbrt/pbrt.h>
#include <pbrt/util/log.h>
#include <pbrt/util/pstd.h>
#include <pbrt/util/vecmath.h>
#include <fstream>

#include <string>

namespace pbrt {

// RenderingCoordinateSystem Definition
enum class RenderingCoordinateSystem { Camera, CameraWorld, World };
std::string ToString(const RenderingCoordinateSystem &);

// BasicPBRTOptions Definition
struct BasicPBRTOptions {
    int seed = 0;
    bool quiet = false;
    bool disablePixelJitter = false, disableWavelengthJitter = false;
    bool disableTextureFiltering = false;
    bool disableImageTextures = false;
    bool forceDiffuse = false;
    bool useGPU = false;
    bool wavefront = false;
    bool interactive = false;
    bool fullscreen = false;
    RenderingCoordinateSystem renderingSpace = RenderingCoordinateSystem::CameraWorld;
};

// PBRTOptions Definition
struct PBRTOptions : BasicPBRTOptions {
    int nThreads = 0;
    LogLevel logLevel = LogLevel::Error;
    std::string logFile;
    bool logUtilization = false;
    bool writePartialImages = false;
    bool recordPixelStatistics = false;
    bool printStatistics = false;
    pstd::optional<int> pixelSamples;
    pstd::optional<int> gpuDevice;
    bool quickRender = false;
    bool upgrade = false;
    std::string imageFile;
    std::string mseReferenceImage, mseReferenceOutput;
    std::string debugStart;
    std::string displayServer;
    pstd::optional<Bounds2f> cropWindow;
    pstd::optional<Bounds2i> pixelBounds;
    pstd::optional<Point2i> pixelMaterial;
    Float displacementEdgeScale = 1;

    // Iridescence
    std::string customLogFile;
    std::shared_ptr<std::ofstream> customLogStream;

    std::string ToString() const;

	bool logPixel() {
        bool useCustom = customLogStream && customLogStream->is_open();
		bool singlePixel =
			pixelBounds.has_value() && pixelBounds.value().Area() == 1;
        bool singleThread = nThreads == 1;
		return useCustom && singlePixel && singleThread;
	}
};

// Options Global Variable Declaration
extern PBRTOptions *Options;

#if defined(PBRT_BUILD_GPU_RENDERER)
#if defined(__CUDACC__)
extern __constant__ BasicPBRTOptions OptionsGPU;
#endif  // __CUDACC__

void CopyOptionsToGPU();
#endif  // PBRT_BUILD_GPU_RENDERER

// Options Inline Functions
PBRT_CPU_GPU inline const BasicPBRTOptions &GetOptions();

PBRT_CPU_GPU inline const BasicPBRTOptions &GetOptions() {
#if defined(PBRT_IS_GPU_CODE)
    return OptionsGPU;
#else
    return *Options;
#endif
}

}  // namespace pbrt

#endif  // PBRT_OPTIONS_H
