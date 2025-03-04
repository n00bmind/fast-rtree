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


template <typename T>
static T Random()
{
    return (T)rand() / ((T)RAND_MAX + 1);
}

static float* MakeRandomPoints( int N )
{
    float* points = (float*)malloc( N * 2 * sizeof(float) );
    for( int i = 0; i < N; i++ )
    {
        points[i*2+0] = Random<float>() * 360.0 - 180.0;
        points[i*2+1] = Random<float>() * 180.0 - 90.0;
    }
    return points;
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
    int64_t ops_ms = (int64_t)(N / elapsed_secs / 1000);

    char ops_buf[64];
    char secs_buf[64];
    printf( "%12lld cycles -- %10d ops in %.6f secs %8.1f ns/op %8lld cycles/op %11lld op/ms\n",
            elapsed_cycles, N, elapsed_secs, ns_op, cycles_op, ops_ms );
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


int main()
{
    constexpr int N = 1'000'000;

    // TODO Keep trying out various min/max values
    // NOTE The 10:1 ratio (max:min) seems to be important!
    RTree<int, 2, 100, 10> tree;
    printf( "RTree with %d:%d (max:min) configuration\n", tree.MaxItemCount, tree.MinItemCount );
    float* points = MakeRandomPoints( N );

    // Compute cycles per second estimation
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


    // TODO Compare with a linear arena allocator
    // TODO When comparing insertion / packing strats, show a sum of area + perimeter for the whole tree & leaf nodes:
    /* "Our secondary comparison metric is the sum of the area and perimeter of the MBRs of the
    R-tree nodes. These measures are good indicators of the number of nodes accessed by a query [6]
    but can be misleading if buffering is not considered [8]. We include these measures as additional
    information and present area and perimeter metrics for both the whole tree (summed over all nodes
    at all levels) and also only for the leaf level. We argue that the leaf level metric is of most interest
    since the non-leaf level nodes will likely be buffered." */
    Bench( "insert", N, [&]( int i )
    {
        float* point = &points[i*2];

        tree.Insert( i, point, point );
        assert( tree.count == i+1 );
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

    Bench( "search-area-1%", 1000, [&]( int i )
    {
        const double p = 0.01;
        float min[2];
        float max[2];
        // TODO Dafuk is rand doing inside the benchmarked code!?
        min[0] = (float)(Random<double>() * 360.0 - 180.0);
        min[1] = (float)(Random<double>() * 180.0 - 90.0);
        max[0] = (float)(min[0] + 360.0 * p);
        max[1] = (float)(min[1] + 180.0 * p);
        int res = 0;
        tree.Search( min, max, IterateRegion, &res );

        // printf( "- %d found items\n", res );
    } );

    Bench( "search-area-5%", 1000, [&]( int i )
    {
        const double p = 0.05;
        float min[2];
        float max[2];
        // TODO Dafuk is rand doing inside the benchmarked code!?
        min[0] = (float)(Random<double>() * 360.0 - 180.0);
        min[1] = (float)(Random<double>() * 180.0 - 90.0);
        max[0] = (float)(min[0] + 360.0 * p);
        max[1] = (float)(min[1] + 180.0 * p);
        int res = 0;
        tree.Search( min, max, IterateRegion, &res );

        // printf( "- %d found items\n", res );
    } );

    Bench( "search-area-10%", 1000, [&]( int i )
    {
        const double p = 0.10;
        float min[2];
        float max[2];
        // TODO Dafuk is rand doing inside the benchmarked code!?
        min[0] = (float)(Random<double>() * 360.0 - 180.0);
        min[1] = (float)(Random<double>() * 180.0 - 90.0);
        max[0] = (float)(min[0] + 360.0 * p);
        max[1] = (float)(min[1] + 180.0 * p);
        int res = 0;
        tree.Search( min, max, IterateRegion, &res );

        // printf( "- %d found items\n", res );
    } );


    return 0;
}
