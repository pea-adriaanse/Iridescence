
#include <gtest/gtest.h>
#include <pbrt/pbrt.h>
#include <pbrt/util/math.h>
#include <pbrt/util/transform.h>
#include <stdio.h>

#include <iri/bxdfs/pyramidBRDF.hpp>

using namespace pbrt;

class PyramidTest : public testing::Test {
  protected:
	Float angleRad;
	PyramidBRDF brdf;
	Vector3f normals[4];

	PyramidTest() {
		this->brdf = PyramidBRDF(1.0, 54.7, 1.0);
		this->angleRad = Radians(54.7);

		// Recalculate normals (private inside brdf)
		Float normalZ = std::cos(Radians(54.7));
		Float normalXY = std::sin(Radians(54.7));

		this->normals[0] = Vector3f(normalXY, 0, normalZ);
		this->normals[1] = Vector3f(0, normalXY, normalZ);
		this->normals[2] = Vector3f(-normalXY, 0, normalZ);
		this->normals[3] = Vector3f(0, -normalXY, normalZ);
	}

	~PyramidTest() {}
};

TEST_F(PyramidTest, pow4Test) {
	EXPECT_EQ(1, PyramidBRDF::pow4(0));
	EXPECT_EQ(4, PyramidBRDF::pow4(1));
	EXPECT_EQ(16, PyramidBRDF::pow4(2));
	EXPECT_EQ(64, PyramidBRDF::pow4(3));
}

TEST_F(PyramidTest, G1) {
	Vector3f wo = Vector3f(0, 0, 1);
	Float shadow = brdf.G1(wo, normals[0]);
	EXPECT_EQ(1.0, shadow);
}

TEST_F(PyramidTest, GenProbability1) {
	constexpr int optionCount = PyramidBRDF::pow4(1);
	ASSERT_EQ(optionCount, 4);

	// Float relativeProb[4 * 4 * 4];
	Float exitProb[optionCount];
	Vector3f outDir[optionCount];

	Vector3f wo = Vector3f(0, 0, 1);
	Float prevProb = 1.0;
	int saveOffset = 0;
	int level = 1;
	int maxLevel = 1;

	brdf.determineProbs(wo, prevProb, saveOffset, exitProb, outDir, level, maxLevel);

	for (int i = 0; i < 4; i++) {
		EXPECT_EQ(outDir[i], Reflect(wo, normals[i]));
		EXPECT_EQ(exitProb[i], brdf.G1(outDir[i], normals[i]));
	}
}