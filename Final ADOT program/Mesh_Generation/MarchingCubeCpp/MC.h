#pragma once

#define MC_IMPLEM_ENABLE

#include <vector>
#include <cmath>
#include <cstdint>

namespace MC
{
	// Optionally define double precision
#ifdef MC_CPP_USE_DOUBLE_PRECISION
	typedef double MC_FLOAT;
#else
	typedef float MC_FLOAT;
#endif

	typedef unsigned int muint;

	typedef struct mcVec3f
	{
	public:
		union 
		{
			MC_FLOAT v[3];
			struct 
			{
				MC_FLOAT x, y, z;
			};
		};
		inline mcVec3f& operator+=(const mcVec3f& r)
		{
			x += r.x; y += r.y; z += r.z;
			return *this;
		}
		inline MC_FLOAT& operator[](int i)
		{
			return v[i];
		}
	} mcVec3f;

	static inline MC_FLOAT mc_internalLength2(const mcVec3f& v)
	{
		return v.x * v.x + v.y * v.y + v.z * v.z;
	}
	static inline MC_FLOAT mc_internalLength(const mcVec3f& v)
	{
		return std::sqrt(mc_internalLength2(v));
	}
	static inline mcVec3f mc_internalNormalize(const mcVec3f& v)
	{
		MC_FLOAT vv = mc_internalLength(v);
		return mcVec3f({{{ v.x / vv, v.y / vv, v.z / vv }}});
	}
	static inline mcVec3f mc_internalCross(const mcVec3f& v1, const mcVec3f& v2)
	{
		return mcVec3f({{{ v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x }}});
	}
	inline mcVec3f operator-(const mcVec3f& l, const mcVec3f r)
	{
		return mcVec3f({{{ l.x - r.x, l.y - r.y, l.z - r.z }}});
	}

	typedef struct mcVec3i
	{
	public:
		union 
		{
			muint v[3];
			struct 
			{
				muint x, y, z;
			};
		};
		inline muint& operator[](int i) { return v[i]; }
	} mcVec3i;

	typedef struct mcMesh
	{
	public:
		std::vector<mcVec3f> vertices;
		std::vector<mcVec3f> normals;
		std::vector<muint> indices;
	} mcMesh;

	void marching_cube(MC_FLOAT* field, muint nx, muint ny, muint nz, mcMesh& outputMesh);
	void setDefaultArraySizes(muint vertSize, muint normSize, muint triSize);
}

