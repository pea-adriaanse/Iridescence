
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
	std::array<Float, NSpectrumSamples> lambdas = {350.0f, 400.0f, 450.0f, 400.0f};

	PyramidTest() {
		this->brdf = PyramidBRDF(1.0, 54.7, 1, false, false, false, 1.0, lambdas, nullptr, "none",
								 Vector3f(0, 0, 0), std::string(""));
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
	EXPECT_EQ(1, pow4(0));
	EXPECT_EQ(4, pow4(1));
	EXPECT_EQ(16, pow4(2));
	EXPECT_EQ(64, pow4(3));
}

TEST_F(PyramidTest, pow4sumTest) {
	EXPECT_EQ(0, pow4sum(0));
	EXPECT_EQ(4, pow4sum(1));
	EXPECT_EQ(20, pow4sum(2));
	EXPECT_EQ(84, pow4sum(3));
	EXPECT_EQ(340, pow4sum(4));
}

TEST_F(PyramidTest, G1) {
	Vector3f wo = Vector3f(0, 0, 1);
	Float shadow = brdf.G1(wo, normals[0]);
	EXPECT_EQ(1.0, shadow);
}

TEST_F(PyramidTest, GenProbability1) {
	constexpr int optionCount = pow4(1);
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

TEST_F(PyramidTest, CalcReflectDist1) {
	// this->brdf = PyramidBRDF(1.0, 54.7, 1.0, false, "none");
	PyramidBRDF::ReflectDist dist;
	Vector3f dir = Vector3f(0, 0, 1);
	brdf.calcReflectDist(&dist, dir);

	EXPECT_EQ(0, dist.exitProbs[0]);
	EXPECT_EQ(0, dist.exitProbs[1]);
	EXPECT_EQ(0, dist.exitProbs[2]);
	EXPECT_EQ(0, dist.exitProbs[3]);

	EXPECT_EQ(0.25, dist.nexitProbs[0]);
	EXPECT_EQ(0.25, dist.nexitProbs[1]);
	EXPECT_EQ(0.25, dist.nexitProbs[2]);
	EXPECT_EQ(0.25, dist.nexitProbs[3]);

	Vector3 outDir0 = Reflect(Vector3f(0, 0, 1), normals[0]);
	Vector3 outDir1 = Reflect(Vector3f(0, 0, 1), normals[1]);
	Vector3 outDir2 = Reflect(Vector3f(0, 0, 1), normals[2]);
	Vector3 outDir3 = Reflect(Vector3f(0, 0, 1), normals[3]);

	float delta0 = 0;
	for (int i = 0; i < 3; i++)
		delta0 += abs(outDir0[i] - dist.outDirs[0][i]);
	EXPECT_LT(delta0, 0.01);
	float delta1 = 0;
	for (int i = 0; i < 3; i++)
		delta1 += abs(outDir1[i] - dist.outDirs[1][i]);
	EXPECT_LT(delta1, 0.01);
	float delta2 = 0;
	for (int i = 0; i < 3; i++)
		delta2 += abs(outDir2[i] - dist.outDirs[2][i]);
	EXPECT_LT(delta2, 0.01);
	float delta3 = 0;
	for (int i = 0; i < 3; i++)
		delta3 += abs(outDir3[i] - dist.outDirs[3][i]);
	EXPECT_LT(delta3, 0.01);
	// TODO dist.brdfs?
}

void expectNear(Vector3f actual, Vector3f expected, float delta) {
	Float diff = 0;
	for (int i = 0; i < 3; i++)
		diff += std::abs(actual[i] - expected[i]);
	EXPECT_NEAR(0, diff, delta);
}

TEST_F(PyramidTest, CalcReflectDist2) {
	this->brdf = PyramidBRDF(1.0, 45.0, 2, false, false, false, 1.0, lambdas, nullptr, "none",
							 Vector3f(0, 0, 0), std::string(""));
	PyramidBRDF::ReflectDist dist;
	Vector3f dir = Vector3f(0, 0, 1);
	brdf.calcReflectDist(&dist, dir);

	EXPECT_EQ(0.25f, dist.nexitProbs[0]);
	expectNear(Vector3f(1.0, 0, 0), dist.outDirs[0], 1e-5);

	unsigned int indexEW = brdf.reflectDistStringToIndex("EW");
	expectNear(Vector3f(0, 0, 1.0f), dist.outDirs[indexEW], 1e-5);
	EXPECT_NEAR(0.25f, dist.exitProbs[indexEW], 1e-5);
}

TEST_F(PyramidTest, ReflectDistStringToIndex) {
	unsigned int index = brdf.reflectDistStringToIndex("E");
	EXPECT_EQ(0, index);
	index = brdf.reflectDistStringToIndex("SEW");
	EXPECT_EQ(70, index);
}

TEST_F(PyramidTest, ReflectDistIndexToString) {
	std::string str = brdf.reflectDistIndexToString(0);
	EXPECT_EQ("E", str);
	str = brdf.reflectDistIndexToString(70);
	EXPECT_EQ("SEW", str);
}

TEST_F(PyramidTest, ReflectDistIndexToFaceIndices) {
	std::vector<unsigned int> path = brdf.reflectDistIndexToFaceIndices(0);
	EXPECT_EQ(1, path.size());
	EXPECT_EQ(0, path[0]);
	path = brdf.reflectDistIndexToFaceIndices(70);
	EXPECT_EQ(3, path.size());
	EXPECT_EQ(3, path[0]);
	EXPECT_EQ(0, path[1]);
	EXPECT_EQ(2, path[2]);
}

TEST_F(PyramidTest, BounceBackIndex) {
	std::string start = "NE";
	unsigned int startIndex = brdf.reflectDistStringToIndex(start);
	std::string next = "NEW";
	unsigned int nextIndex = brdf.reflectDistStringToIndex(next);
	unsigned int backBounceIndex = brdf.getBackbounceIndex(startIndex, nextIndex);
	std::string backBounce = brdf.reflectDistIndexToString(backBounceIndex);
	EXPECT_EQ(backBounce, "NEWEN");
}