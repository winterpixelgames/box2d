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

#include "box2d/b2_sdf_shape.h"
#include "box2d/b2_block_allocator.h"

#include <new>

b2Shape* b2SDFShape::Clone(b2BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b2SDFShape));
	b2SDFShape* clone = new (mem) b2SDFShape;
	*clone = *this;
	return clone;
}

int32 b2SDFShape::GetChildCount() const
{
	return 1;
}

bool b2SDFShape::TestPoint(const b2Transform& transform, const b2Vec2& p) const
{
	b2Vec2 center = transform.p + b2Mul(transform.q, m_p);
	return m_map(center) <= 0;
}

bool b2SDFShape::RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
							const b2Transform& transform, int32 childIndex) const
{
	B2_NOT_USED(childIndex);

	//TODO: figure out
	return true;
}

void b2SDFShape::ComputeAABB(b2AABB* aabb, const b2Transform& transform, int32 childIndex) const
{
	B2_NOT_USED(childIndex);

	//TODO: can we?
	//b2Vec2 p = transform.p + b2Mul(transform.q, m_p);
	aabb->lowerBound.Set(FLT_MIN, FLT_MIN);
	aabb->upperBound.Set(FLT_MAX, FLT_MAX);
}

void b2SDFShape::ComputeMass(b2MassData* massData, float density) const
{
	//TODO: figure out
	massData->mass = 1.0;
	massData->center = m_p;

	// inertia about the local origin
	massData->I = 0.0;
}
