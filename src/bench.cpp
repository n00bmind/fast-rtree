#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cfloat>
#include <chrono>
#include <intrin.h>
#include <immintrin.h>

#define FAST_RTREE_IMPLEMENTATION 1
#include "rtree.h"
using namespace fast_rtree;


struct MemoryArena
{
    uint8_t* base;
    size_t used;
    size_t capacity;
};
MemoryArena globalArena = {};

void InitArena( MemoryArena* arena, size_t capacityBytes )
{
    RTREE_ASSERT( arena->base == nullptr );
    arena->base = (uint8_t*)malloc( capacityBytes );
    arena->used = 0;
    arena->capacity = capacityBytes;
}

void ClearArena( MemoryArena* arena )
{
    arena->used = 0;
}

void* ArenaAlloc( MemoryArena* arena, size_t sizeBytes )
{
    if( arena->used + sizeBytes < arena->capacity )
    {
        void* p = arena->base + arena->used;
        arena->used += sizeBytes;
        return p;
    }
    else
    {
        RTREE_ASSERT( arena->used + sizeBytes < arena->capacity );
        return nullptr;
    }
}

void* RTreeAlloc( size_t sizeBytes )
{
    return ArenaAlloc( &globalArena, sizeBytes );
}

void ArenaFree( MemoryArena* arena, void* p )
{
    // no-op
}

void RTreeFree( void* p )
{
    ArenaFree( &globalArena, p );
}

static float RandomFloat()
{
    return (float)rand() / ((float)RAND_MAX + 1);
}

static float* MakeRandomPoints( int N )
{
    float* points = (float*)malloc( N * 2 * sizeof(float) );
    for( int i = 0; i < N; i++ )
    {
        // NOTE Map as latitude / longitude
        points[i*2+0] = RandomFloat() * 360.0 - 180.0;
        points[i*2+1] = RandomFloat() * 180.0 - 90.0;
    }
    return points;
}

// All the Hilbert ordering routines taken from https://github.com/tidwall/rtree.c
uint32_t interleave( uint32_t x )
{
    x = (x | (x << 8)) & 0x00FF00FF;
    x = (x | (x << 4)) & 0x0F0F0F0F;
    x = (x | (x << 2)) & 0x33333333;
    x = (x | (x << 1)) & 0x55555555;
    return x;
}

uint32_t hilbertXYToIndex_logarithmic( uint32_t x, uint32_t y )
{
    uint32_t A, B, C, D;

    // Initial prefix scan round, prime with x and y
    {
        uint32_t a = x ^ y;
        uint32_t b = 0xFFFF ^ a;
        uint32_t c = 0xFFFF ^ (x | y);
        uint32_t d = x & (y ^ 0xFFFF);

        A = a | (b >> 1);
        B = (a >> 1) ^ a;

        C = ((c >> 1) ^ (b & (d >> 1))) ^ c;
        D = ((a & (c >> 1)) ^ (d >> 1)) ^ d;
    }

    {
        uint32_t a = A;
        uint32_t b = B;
        uint32_t c = C;
        uint32_t d = D;

        A = ((a & (a >> 2)) ^ (b & (b >> 2)));
        B = ((a & (b >> 2)) ^ (b & ((a ^ b) >> 2)));

        C ^= ((a & (c >> 2)) ^ (b & (d >> 2)));
        D ^= ((b & (c >> 2)) ^ ((a ^ b) & (d >> 2)));
    }

    {
        uint32_t a = A;
        uint32_t b = B;
        uint32_t c = C;
        uint32_t d = D;

        A = ((a & (a >> 4)) ^ (b & (b >> 4)));
        B = ((a & (b >> 4)) ^ (b & ((a ^ b) >> 4)));

        C ^= ((a & (c >> 4)) ^ (b & (d >> 4)));
        D ^= ((b & (c >> 4)) ^ ((a ^ b) & (d >> 4)));
    }

    // Final round and projection
    {
        uint32_t a = A;
        uint32_t b = B;
        uint32_t c = C;
        uint32_t d = D;

        C ^= ((a & (c >> 8)) ^ (b & (d >> 8)));
        D ^= ((b & (c >> 8)) ^ ((a ^ b) & (d >> 8)));
    }

    // Undo transformation prefix scan
    uint32_t a = C ^ (C >> 1);
    uint32_t b = D ^ (D >> 1);

    // Recover index bits
    uint32_t i0 = x ^ y;
    uint32_t i1 = b | (0xFFFF ^ (i0 | a));

    return (interleave(i1) << 1) | interleave(i0);
}

uint32_t hilbert( float lat, float lon )
{
    uint32_t x = ((lon + 180.0) / 360.0) * 0xFFFF;
    uint32_t y = ((lat + 90.0) / 180.0) * 0xFFFF;
    return hilbertXYToIndex_logarithmic( x, y );
}

int HilbertCompare( const void *a, const void *b )
{
    float *p1 = (float*)a;
    float *p2 = (float*)b;
    uint32_t h1 = hilbert( p1[1] , p1[0] );
    uint32_t h2 = hilbert( p2[1] , p2[0] );

    return (h1 < h2) ? -1 : ((h1 > h2) ? 1 : 0);
}

void HilbertSort( float* points, int N )
{
    qsort( points, N, sizeof(float) * 2, HilbertCompare );
}


static double g_cyclesPerSecond;

template <typename Callable>
void Bench( char const* name, int N, Callable&& code )
{
    printf( "%-20s ", name );

    uint64_t begin = __rdtsc();
    for( int i = 0; i < N; i++ ) {
        code( i );
    }
    uint64_t end = __rdtsc();

    uint64_t elapsed_cycles = end - begin;
    double elapsed_secs = elapsed_cycles / g_cyclesPerSecond;
    double ns_op = elapsed_secs / (double)N * 1e9;
    int64_t cycles_op = (int64_t)(elapsed_cycles / (double)N);

    char ops_buf[64];
    char secs_buf[64];
    printf( "%12lld cycles -- %10d ops in %.6f secs   / %8lld cycles/op -- %8.1f ns/op\n",
            elapsed_cycles, N, elapsed_secs, cycles_op, ns_op );
}

struct IterateItemCtx
{
    float* point;
    int data;
    int count;
};

static bool IterateItem( const float* min, const float* max, const void* itemData, void* userdata )
{
    IterateItemCtx* ctx = (IterateItemCtx*)userdata;
    if( *(int*)itemData == ctx->data )
    {
        // assert( memcmp( min, ctx->point, sizeof(float)*2 ) == 0 );
        // assert( memcmp( max, ctx->point, sizeof(float)*2 ) == 0 );
        // ctx->count++;
        return false;
    }
    return true;
}

static bool IterateRegion( const float* min, const float* max, const void* itemData, void* userdata )
{
    (*(int*)userdata)++;
    return true;
}


// TODO Seems like we may wanna start moving towards more of a "repetition tester" model?
template <int MaxItems, int MinItems>
void TestRandomSet( int N, bool hilbertOrdered )
{
    ClearArena( &globalArena );

    // TODO Keep trying out various min/max values
    // NOTE The 10:1 ratio (max:min) seems to be important!
#if 0
    RTree<int, 2, MaxItems, MinItems> tree;
#else
    RTree<int, 2, MaxItems, MinItems> tree( RTreeAlloc, RTreeFree );
#endif
    printf( "\n\nRTree with %d:%d (max:min) configuration\n", tree.MaxItemCount, tree.MinItemCount );

    float* points = MakeRandomPoints( N );
    if( hilbertOrdered )
        HilbertSort( points, N );
    printf( "Dataset contains %d points%s\n", N, hilbertOrdered ? " (Hilbert ordered)" : "" );

    // Compute cycles per second estimation
    // NOTE This *shouldnt* vary across tests on modern processors
    using namespace std::chrono;

    double elapsed_secs;
    auto t0 = high_resolution_clock::now();
    uint64_t c0 = __rdtsc();
    while( true )
    {
        _mm_pause();

        auto t1 = high_resolution_clock::now();
        elapsed_secs = duration_cast<duration<double>>(t1 - t0).count();
        if( elapsed_secs >= 1.0 )
            break;
    }
    uint64_t c1 = __rdtsc();
    uint64_t elapsed_cycles = c1 - c0;

    g_cyclesPerSecond = (double)elapsed_cycles / elapsed_secs;
    printf( "Cycles per second (est.): %.1f\n\n", g_cyclesPerSecond );


    // TODO When comparing insertion / packing strats, show a sum of area + perimeter for the whole tree & leaf nodes:
    /* "Our secondary comparison metric is the sum of the area and perimeter of the MBRs of the
    R-tree nodes. These measures are good indicators of the number of nodes accessed by a query [6]
    [...]. We include these measures as additional
    information and present area and perimeter metrics for both the whole tree (summed over all nodes
    at all levels) and also only for the leaf level. We argue that the leaf level metric is of most interest
    since the non-leaf level nodes will likely be buffered." */
    Bench( "insert", N, [&]( int i )
    {
        float* point = &points[i*2];

        tree.Insert( i, point, point );
        // assert( tree.count == i+1 );
    } );

    // TODO Test searching before and after reallocating all nodes so that depth-first traversing is linear in memory
    // TODO Also, it may be interesting to try out other approaches that take advantage of the memory layout of the tree,
    // f.e. it should be interesting to try out a breadth-first search, given that all of a node's rectangle are necessarily
    // linear in memory!? (so test them all and store the results, before jumping nodes). If we further refine the layout
    // so that leaves are all contiguous and in breadth-first order, this may be faster!?
    Bench( "search-item", N, [&]( int i )
    {
        float *p = &points[i*2];
        float point[2] = { (float)p[0], (float)p[1] };

        IterateItemCtx ctx;
        ctx.point = point;
        ctx.data = i;
        ctx.count = 0;
        tree.Search( point, point, IterateItem, &ctx );
        // assert( ctx.count == 1 );
    } );


    int areaSearchCount = N / 1000;
    float* searchPoints = MakeRandomPoints( areaSearchCount );

    Bench( "search-area-1%", areaSearchCount, [&]( int i )
    {
        constexpr double pcnt = 0.01;
        float *p = &points[i*2];
        float min[2] = { p[0], p[1] };
        float max[2] = { (float)(min[0] + 360.0 * pcnt), (float)(min[1] + 180.0 * pcnt) };

        int res = 0;
        tree.Search( min, max, IterateRegion, &res );
        // printf( "- %d found items\n", res );
    } );

    free( searchPoints );
    searchPoints = MakeRandomPoints( areaSearchCount );

    Bench( "search-area-5%", areaSearchCount, [&]( int i )
    {
        constexpr double pcnt = 0.05;
        float *p = &points[i*2];
        float min[2] = { p[0], p[1] };
        float max[2] = { (float)(min[0] + 360.0 * pcnt), (float)(min[1] + 180.0 * pcnt) };

        int res = 0;
        tree.Search( min, max, IterateRegion, &res );
        // printf( "- %d found items\n", res );
    } );

    free( searchPoints );
    searchPoints = MakeRandomPoints( areaSearchCount );

    Bench( "search-area-10%", areaSearchCount, [&]( int i )
    {
        constexpr double pcnt = 0.10;
        float *p = &points[i*2];
        float min[2] = { p[0], p[1] };
        float max[2] = { (float)(min[0] + 360.0 * pcnt), (float)(min[1] + 180.0 * pcnt) };

        int res = 0;
        tree.Search( min, max, IterateRegion, &res );
        // printf( "- %d found items\n", res );
    } );

    free( searchPoints );
    free( points );
}

int main()
{
    InitArena( &globalArena, 64 * 1024 * 1024 );

    constexpr int N = 1'000'000;

    TestRandomSet<64, 7>( N, false );
    TestRandomSet<100, 10>( N, false );
    TestRandomSet<64, 7>( N, true );
    TestRandomSet<100, 10>( N, true );

    return 0;
}
