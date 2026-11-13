// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box3d/base.h"
#include "box3d/box3d.h"
#include "box3d/collision.h"

#include <stdint.h>

// The clock backs the world profile, so a broken clock silently reports nonsense timings. These
// checks only use the public API and hold on every platform, including Emscripten where the
// timestamp comes from emscripten_get_now instead of clock_gettime.

// b3GetTicks is the raw clock. It must advance, and never go backwards, which is what the
// monotonic clock is for. The resolution differs per platform, so the wait has to be generous
// enough for a millisecond clock under Emscripten.
static int TicksAdvance( void )
{
	uint64_t ticks = b3GetTicks();
	uint64_t previous = ticks;

	// A millisecond clock needs a few milliseconds to move, and the host may be loaded.
	for ( int i = 0; i < 100; ++i )
	{
		b3Sleep( 1 );
		uint64_t now = b3GetTicks();
		ENSURE( now >= previous );
		previous = now;
		if ( now > ticks )
		{
			break;
		}
	}

	ENSURE( previous > ticks );
	return 0;
}

// b3GetMilliseconds measures from a start tick. Sleeping is the only portable way to make time
// pass, and sleep plus scheduling overhead can overshoot, so this checks the lower bound and a
// loose upper bound rather than an exact value.
static int MillisecondsElapsed( void )
{
	uint64_t ticks = b3GetTicks();

	// Measuring from a tick just read gives nearly nothing. The bound is loose because the
	// host can deschedule the thread between the two calls.
	ENSURE( b3GetMilliseconds( ticks ) < 100.0f );

	b3Sleep( 20 );

	float ms = b3GetMilliseconds( ticks );
	ENSURE( ms >= 19.0f );
	ENSURE( ms < 5000.0f );
	return 0;
}

// b3GetMillisecondsAndReset reports the same interval as b3GetMilliseconds and then restarts the
// measurement, so a second call covers only the time since the first one. The sleep is long
// enough that a descheduled thread between the two calls is unlikely to reach it.
static int MillisecondsAndReset( void )
{
	uint64_t ticks = b3GetTicks();

	b3Sleep( 100 );

	float first = b3GetMillisecondsAndReset( &ticks );
	ENSURE( first >= 95.0f );
	ENSURE( first < 5000.0f );

	// The reset means the second interval starts over, so it does not include the sleep.
	float second = b3GetMillisecondsAndReset( &ticks );
	ENSURE( second >= 0.0f );
	ENSURE( second < 95.0f );
	return 0;
}

// Stepping a world fills the profile from the clock. The phase times are all measured with the
// same timer, so they must be non-negative and add up to no more than the total.
static int WorldProfileTiming( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -10.0f, 0.0f };

	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) );

	b3BodyDef groundBodyDef = b3DefaultBodyDef();
	groundBodyDef.position = (b3Pos){ 0.0f, -10.0f, 0.0f };
	b3BodyId groundId = b3CreateBody( worldId, &groundBodyDef );

	b3BoxHull groundBox = b3MakeBoxHull( 50.0f, 10.0f, 50.0f );
	b3ShapeDef groundShapeDef = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShapeDef, &groundBox.base );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 4.0f, 0.0f };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3BoxHull box = b3MakeCubeHull( 1.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	for ( int i = 0; i < 8; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3Profile profile = b3World_GetProfile( worldId );
	ENSURE( profile.step >= 0.0f );
	ENSURE( profile.pairs >= 0.0f );
	ENSURE( profile.collide >= 0.0f );
	ENSURE( profile.solve >= 0.0f );
	ENSURE( profile.sensors >= 0.0f );

	// The whole step cannot be shorter than the phases measured inside it.
	float phases = profile.pairs + profile.collide + profile.solve + profile.sensors;
	ENSURE( phases <= profile.step + 1.0f );

	b3DestroyWorld( worldId );
	return 0;
}

int TimerTest( void )
{
	RUN_SUBTEST( TicksAdvance );
	RUN_SUBTEST( MillisecondsElapsed );
	RUN_SUBTEST( MillisecondsAndReset );
	RUN_SUBTEST( WorldProfileTiming );
	return 0;
}
