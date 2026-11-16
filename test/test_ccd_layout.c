// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

// Continuous collision and explosion effects must not depend on the internal ordering of the
// broad-phase tree. Moving the same static bodies in different orders produces trees with equal
// ids, bounds, and moved marks but a different visit order. Each test builds two worlds that
// differ only in the order the static bodies are moved, witnesses the differing visit order from
// the public API, then requires the continuous collision result to agree.

#include "test_macros.h"

#include "box3d/box3d.h"

enum
{
	BOX_COUNT = 16,
	SENSOR_COUNT = 12,
	SEED_COUNT = 64,
};

// Records the shape ids an overlap query visits, in order.
typedef struct VisitCapture
{
	b3ShapeId ids[64];
	int count;
} VisitCapture;

static bool VisitFcn( b3ShapeId shapeId, void* context )
{
	VisitCapture* capture = context;
	if ( capture->count < 64 )
	{
		capture->ids[capture->count++] = shapeId;
	}
	return true;
}

// Cross-platform deterministic random generator matching shared/utils.h.
static uint32_t s_rng;

static int RandomInt( void )
{
	uint32_t x = s_rng;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	s_rng = x;
	return (int)( x % ( 32767 + 1 ) );
}

static float RandomFloat( void )
{
	float r = (float)( RandomInt() & 32767 );
	return r / 32767.0f;
}

static bool WitnessOrderDiffers( const VisitCapture* a, const VisitCapture* b )
{
	ENSURE( a->count == b->count );
	for ( int i = 0; i < a->count; ++i )
	{
		if ( a->ids[i].index1 != b->ids[i].index1 )
		{
			return true;
		}
	}
	return false;
}

// Build one world of static boxes from the given positions, then move every box 50m off to the
// side and back. The bodies are moved in ascending order when reverseOrder is false and descending
// when it is true, which re-inserts the proxies in opposite orders and produces a different tree.
// The witness records the public visit order of a query that overlaps every box.
static void BuildLayoutWorld( b3WorldId worldId, const b3Pos* positions, bool reverseOrder, VisitCapture* witness )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	b3BodyId bodies[BOX_COUNT];
	for ( int i = 0; i < BOX_COUNT; ++i )
	{
		bodyDef.position = positions[i];
		bodies[i] = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( bodies[i], &shapeDef, &box.base );
	}

	b3Vec3 offset = { 50.0f, 0.0f, 0.0f };
	for ( int step = 0; step < BOX_COUNT; ++step )
	{
		int i = reverseOrder ? BOX_COUNT - 1 - step : step;
		b3Pos home = positions[i];
		b3Body_SetTransform( bodies[i], b3OffsetPos( home, offset ), b3Quat_identity );
		b3Body_SetTransform( bodies[i], home, b3Quat_identity );
	}

	b3AABB boxAroundBoxes = { { -0.6f, -1.0f, -0.6f }, { 3.6f, 1.0f, 3.6f } };
	b3World_OverlapAABB( worldId, boxAroundBoxes, b3DefaultQueryFilter(), VisitFcn, witness );
}

// A fast bullet swept over a field of random boxes. The box hit last (the lowest time of impact)
// must be the same, and so must the final transform and the iteration counters, regardless of the
// tree layout.
static int ContinuousLayoutImpactTest( void )
{
	for ( int seed = 0; seed < SEED_COUNT; ++seed )
	{
		s_rng = 12345 + seed * 1000;

		b3Pos positions[BOX_COUNT];
		for ( int i = 0; i < BOX_COUNT; ++i )
		{
			positions[i] = (b3Pos){ 3.0f * RandomFloat(), 0.0f, 3.0f * RandomFloat() };
		}

		b3WorldDef worldDef = b3DefaultWorldDef();
		worldDef.enableSleep = false;
		worldDef.workerCount = 1;

		b3WorldId worldA = b3CreateWorld( &worldDef );
		b3WorldId worldB = b3CreateWorld( &worldDef );

		VisitCapture witnessA = { 0 };
		VisitCapture witnessB = { 0 };
		BuildLayoutWorld( worldA, positions, false, &witnessA );
		BuildLayoutWorld( worldB, positions, true, &witnessB );
		ENSURE( WitnessOrderDiffers( &witnessA, &witnessB ) );

		b3BodyDef bulletDef = b3DefaultBodyDef();
		bulletDef.type = b3_dynamicBody;
		bulletDef.isBullet = true;
		bulletDef.position = (b3Pos){ 1.5f, 4.0f, 1.5f };
		bulletDef.linearVelocity = (b3Vec3){ 0.0f, -150.0f, 0.0f };
		bulletDef.gravityScale = 0.0f;

		b3ShapeDef sphereDef = b3DefaultShapeDef();
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };

		b3BodyId bulletA = b3CreateBody( worldA, &bulletDef );
		b3CreateSphereShape( bulletA, &sphereDef, &sphere );
		b3BodyId bulletB = b3CreateBody( worldB, &bulletDef );
		b3CreateSphereShape( bulletB, &sphereDef, &sphere );

		float dt = 1.0f / 60.0f;
		for ( int i = 0; i < 10; ++i )
		{
			b3World_Step( worldA, dt, 4 );
			b3World_Step( worldB, dt, 4 );
		}

		b3WorldTransform transformA = b3Body_GetTransform( bulletA );
		b3WorldTransform transformB = b3Body_GetTransform( bulletB );

		ENSURE_SMALL( transformA.p.x - transformB.p.x, 1e-6f );
		ENSURE_SMALL( transformA.p.y - transformB.p.y, 1e-6f );
		ENSURE_SMALL( transformA.p.z - transformB.p.z, 1e-6f );
		ENSURE_SMALL( transformA.q.v.x - transformB.q.v.x, 1e-6f );
		ENSURE_SMALL( transformA.q.v.y - transformB.q.v.y, 1e-6f );
		ENSURE_SMALL( transformA.q.v.z - transformB.q.v.z, 1e-6f );
		ENSURE_SMALL( transformA.q.s - transformB.q.s, 1e-6f );

		b3Counters countersA = b3World_GetCounters( worldA );
		b3Counters countersB = b3World_GetCounters( worldB );
		ENSURE( countersA.distanceIterations == countersB.distanceIterations );
		ENSURE( countersA.pushBackIterations == countersB.pushBackIterations );
		ENSURE( countersA.rootIterations == countersB.rootIterations );

		b3DestroyWorld( worldA );
		b3DestroyWorld( worldB );
	}
	return 0;
}

// A pre-solve callback that rejects every candidate. The sequence of rejected shapes and the final
// transform must agree across layouts.
typedef struct PreSolveLog
{
	int shapeIds[64];
	int count;
} PreSolveLog;

static bool RejectingPreSolveFcn( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3Pos point, b3Vec3 normal, void* context )
{
	(void)shapeIdB;
	(void)point;
	(void)normal;
	PreSolveLog* log = context;
	if ( log->count < 64 )
	{
		log->shapeIds[log->count++] = shapeIdA.index1 - 1;
	}
	return false;
}

static int ContinuousLayoutPreSolveTest( void )
{
	for ( int seed = 0; seed < SEED_COUNT; ++seed )
	{
		s_rng = 12345 + seed * 1000;

		b3Pos positions[BOX_COUNT];
		for ( int i = 0; i < BOX_COUNT; ++i )
		{
			positions[i] = (b3Pos){ 3.0f * RandomFloat(), 0.0f, 3.0f * RandomFloat() };
		}

		b3WorldDef worldDef = b3DefaultWorldDef();
		worldDef.enableSleep = false;
		worldDef.workerCount = 1;

		b3WorldId worldA = b3CreateWorld( &worldDef );
		b3WorldId worldB = b3CreateWorld( &worldDef );

		VisitCapture witnessA = { 0 };
		VisitCapture witnessB = { 0 };
		BuildLayoutWorld( worldA, positions, false, &witnessA );
		BuildLayoutWorld( worldB, positions, true, &witnessB );
		ENSURE( WitnessOrderDiffers( &witnessA, &witnessB ) );

		b3BodyDef bulletDef = b3DefaultBodyDef();
		bulletDef.type = b3_dynamicBody;
		bulletDef.isBullet = true;
		bulletDef.position = (b3Pos){ 1.5f, 4.0f, 1.5f };
		bulletDef.linearVelocity = (b3Vec3){ 0.0f, -150.0f, 0.0f };
		bulletDef.gravityScale = 0.0f;

		b3ShapeDef sphereDef = b3DefaultShapeDef();
		sphereDef.enablePreSolveEvents = true;
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };

		b3BodyId bulletA = b3CreateBody( worldA, &bulletDef );
		b3CreateSphereShape( bulletA, &sphereDef, &sphere );
		b3BodyId bulletB = b3CreateBody( worldB, &bulletDef );
		b3CreateSphereShape( bulletB, &sphereDef, &sphere );

		PreSolveLog logA = { 0 };
		PreSolveLog logB = { 0 };
		b3World_SetPreSolveCallback( worldA, RejectingPreSolveFcn, &logA );
		b3World_SetPreSolveCallback( worldB, RejectingPreSolveFcn, &logB );

		float dt = 1.0f / 60.0f;
		for ( int i = 0; i < 10; ++i )
		{
			b3World_Step( worldA, dt, 4 );
			b3World_Step( worldB, dt, 4 );
		}

		// The rejecting callback runs once per candidate, in the same order regardless of layout.
		ENSURE( logA.count == logB.count );
		ENSURE( logA.count > 0 );
		for ( int i = 0; i < logA.count; ++i )
		{
			ENSURE( logA.shapeIds[i] == logB.shapeIds[i] );
		}

		b3WorldTransform transformA = b3Body_GetTransform( bulletA );
		b3WorldTransform transformB = b3Body_GetTransform( bulletB );
		ENSURE_SMALL( transformA.p.x - transformB.p.x, 1e-6f );
		ENSURE_SMALL( transformA.p.y - transformB.p.y, 1e-6f );
		ENSURE_SMALL( transformA.p.z - transformB.p.z, 1e-6f );

		b3DestroyWorld( worldA );
		b3DestroyWorld( worldB );
	}
	return 0;
}

// A fast bullet that sweeps through more sensors than fit in the kept buffer. The sensors kept and
// reported must be the same regardless of the tree layout.
static int ContinuousLayoutSensorTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.enableSleep = false;
	worldDef.workerCount = 1;

	b3WorldId worldA = b3CreateWorld( &worldDef );
	b3WorldId worldB = b3CreateWorld( &worldDef );

	// A vertical stack of sensors along the bullet's path. Their ids are ascending downward, so
	// the bullet visits them in ascending id order but more than B2_MAX_CONTINUOUS_SENSOR_HITS
	// fit, forcing a selection that must not depend on tree order.
	b3Pos sensorPositions[SENSOR_COUNT];
	for ( int i = 0; i < SENSOR_COUNT; ++i )
	{
		sensorPositions[i] = (b3Pos){ 0.0f, 2.9f - 0.3f * i, 0.0f };
	}

	b3BodyDef sensorBodyDef = b3DefaultBodyDef();
	sensorBodyDef.type = b3_staticBody;

	b3ShapeDef sensorDef = b3DefaultShapeDef();
	sensorDef.isSensor = true;
	sensorDef.enableSensorEvents = true;

	b3Sphere sensorSphere = { { 0.0f, 0.0f, 0.0f }, 0.1f };

	b3BodyId sensorsA[SENSOR_COUNT];
	b3BodyId sensorsB[SENSOR_COUNT];
	for ( int i = 0; i < SENSOR_COUNT; ++i )
	{
		sensorBodyDef.position = sensorPositions[i];
		sensorsA[i] = b3CreateBody( worldA, &sensorBodyDef );
		b3CreateSphereShape( sensorsA[i], &sensorDef, &sensorSphere );
		sensorsB[i] = b3CreateBody( worldB, &sensorBodyDef );
		b3CreateSphereShape( sensorsB[i], &sensorDef, &sensorSphere );
	}

	// Move the sensors off and back in opposite orders to produce different layouts.
	b3Vec3 offset = { 50.0f, 0.0f, 0.0f };
	for ( int step = 0; step < SENSOR_COUNT; ++step )
	{
		int ia = step;
		int ib = SENSOR_COUNT - 1 - step;
		b3Body_SetTransform( sensorsA[ia], b3OffsetPos( sensorPositions[ia], offset ), b3Quat_identity );
		b3Body_SetTransform( sensorsA[ia], sensorPositions[ia], b3Quat_identity );
		b3Body_SetTransform( sensorsB[ib], b3OffsetPos( sensorPositions[ib], offset ), b3Quat_identity );
		b3Body_SetTransform( sensorsB[ib], sensorPositions[ib], b3Quat_identity );
	}

	b3AABB boxAroundSensors = { { -1.0f, -1.0f, -1.0f }, { 1.0f, 3.5f, 1.0f } };
	VisitCapture witnessA = { 0 };
	VisitCapture witnessB = { 0 };
	b3World_OverlapAABB( worldA, boxAroundSensors, b3DefaultQueryFilter(), VisitFcn, &witnessA );
	b3World_OverlapAABB( worldB, boxAroundSensors, b3DefaultQueryFilter(), VisitFcn, &witnessB );
	ENSURE( WitnessOrderDiffers( &witnessA, &witnessB ) );

	b3BodyDef bulletDef = b3DefaultBodyDef();
	bulletDef.type = b3_dynamicBody;
	bulletDef.isBullet = true;
	bulletDef.position = (b3Pos){ 0.0f, 5.0f, 0.0f };
	bulletDef.linearVelocity = (b3Vec3){ 0.0f, -150.0f, 0.0f };
	bulletDef.gravityScale = 0.0f;

	b3ShapeDef sphereDef = b3DefaultShapeDef();
	sphereDef.enableSensorEvents = true;
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.1f };

	b3BodyId bulletA = b3CreateBody( worldA, &bulletDef );
	b3CreateSphereShape( bulletA, &sphereDef, &sphere );
	b3BodyId bulletB = b3CreateBody( worldB, &bulletDef );
	b3CreateSphereShape( bulletB, &sphereDef, &sphere );

	// Collect the union of begin events across steps. Each step sees the subset of the stack the
	// bullet passed through, which must be the same set in the same order.
	int sequenceA[64];
	int sequenceB[64];
	int sequenceCountA = 0;
	int sequenceCountB = 0;

	float dt = 1.0f / 60.0f;
	for ( int i = 0; i < 3; ++i )
	{
		b3World_Step( worldA, dt, 1 );
		b3World_Step( worldB, dt, 1 );

		b3SensorEvents eventsA = b3World_GetSensorEvents( worldA );
		b3SensorEvents eventsB = b3World_GetSensorEvents( worldB );

		for ( int j = 0; j < eventsA.beginCount && sequenceCountA < 64; ++j )
		{
			sequenceA[sequenceCountA++] = eventsA.beginEvents[j].sensorShapeId.index1 - 1;
		}
		for ( int j = 0; j < eventsB.beginCount && sequenceCountB < 64; ++j )
		{
			sequenceB[sequenceCountB++] = eventsB.beginEvents[j].sensorShapeId.index1 - 1;
		}
	}

	// Both worlds report the same sensors in the same order. The selection over the full stack
	// must agree even though each step visits more than the kept buffer.
	ENSURE( sequenceCountA == sequenceCountB );
	ENSURE( sequenceCountA > 0 );
	for ( int i = 0; i < sequenceCountA; ++i )
	{
		ENSURE( sequenceA[i] == sequenceB[i] );
	}

	b3DestroyWorld( worldA );
	b3DestroyWorld( worldB );
	return 0;
}

int CcdLayoutTest( void )
{
	RUN_SUBTEST( ContinuousLayoutImpactTest );
	RUN_SUBTEST( ContinuousLayoutPreSolveTest );
	RUN_SUBTEST( ContinuousLayoutSensorTest );

	return 0;
}