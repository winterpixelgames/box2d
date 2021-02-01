// MIT License

// Copyright (c) 2019 Erin Catto
// Copyright (c) 2021 Jordan Schidlowsky

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef B2_SDF_SHAPE_H
#define B2_SDF_SHAPE_H

#include "b2_api.h"
#include "b2_shape.h"
#include <functional> // use functional for prototyping

/// An sdf shape
class B2_API b2SDFShape : public b2Shape
{
public:
	b2SDFShape();

	/// Implement b2Shape.
	b2Shape* Clone(b2BlockAllocator* allocator) const override;

	/// @see b2Shape::GetChildCount
	int32 GetChildCount() const override;

	/// Implement b2Shape.
	bool TestPoint(const b2Transform& transform, const b2Vec2& p) const override;

	/// Implement b2Shape.
	bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
				const b2Transform& transform, int32 childIndex) const override;

	/// @see b2Shape::ComputeAABB
	void ComputeAABB(b2AABB* aabb, const b2Transform& transform, int32 childIndex) const override;

	/// @see b2Shape::ComputeMass
	void ComputeMass(b2MassData* massData, float density) const override;

	// SDF derivative
	b2Vec2 Gradient(const b2Vec2& p) const;

	/// Position
	b2Vec2 m_p;

	// SDF function
	std::function<float(const b2Vec2&)> m_map;
};

inline b2SDFShape::b2SDFShape()
{
	m_type = e_sdf;
	m_radius = b2_polygonRadius;
	m_p.SetZero();
	m_map = [](const b2Vec2& p) { 
		b2Assert(false);  // should always be set by something else.
		return p.Length(); // default single point?
	};
}

inline b2Vec2 b2SDFShape::Gradient(const b2Vec2& p) const 
{
	//const float EPS_N = 0.01; // TODO: figure out epsilon
	const float EPS_N = 1.00; // TODO: figure out epsilon
	return b2Vec2( m_map(b2Vec2(p.x+EPS_N, p.y)) - m_map(b2Vec2(p.x-EPS_N, p.y)),
	 m_map(b2Vec2(p.x, p.y+EPS_N)) - m_map(b2Vec2(p.x, p.y-EPS_N)) );
}

#endif
