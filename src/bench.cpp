#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cfloat>
#include <ctime>

#define FAST_RTREE_IMPLEMENTATION 1
#include "rtree.h"
using namespace fast_rtree;


template <typename T>
static double Random()
{
    return (T)rand() / ((T)RAND_MAX + 1);
}

static double* MakeRandomPoints( int N )
{
    double* points = (double*)malloc( N * 2 * sizeof(double) );
    for( int i = 0; i < N; i++ )
    {
        points[i*2+0] = Random<double>() * 360.0 - 180.0;
        points[i*2+1] = Random<double>() * 180.0 - 90.0;
    }
    return points;
}

template <typename Callable>
void Bench( char const* name, int N, Callable&& code )
{
    printf( "%-14s ", name );

    // TODO Use rdtsc instead
    clock_t begin = clock();
    for( int i = 0; i < N; i++ ) {
        code( i );
    }
    clock_t end = clock();

    double elapsed_secs = (double)(end - begin) / CLOCKS_PER_SEC;
    double ns_op = elapsed_secs / (double)N * 1e9;
    int64_t ops_sec = (int64_t)(N / elapsed_secs);

    char ops_buf[64];
    char secs_buf[64];
    printf( "%10d ops in %.3f secs %8.1f ns/op %11lld op/sec\n", N, elapsed_secs, ns_op, ops_sec );
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

    RTree<int> tree;
    double* points = MakeRandomPoints( N );

    Bench( "insert", N, [&]( int i )
    {
        double* p = &points[i*2];
        float point[2] = { (float)p[0], (float)p[1] };

        tree.Insert( i, point, point );
        assert( tree.count == i+1 );
    } );

    // TODO Test searching before and after reallocating all nodes so that depth-first traversing is linear in memory

    Bench( "search-item", N, [&]( int i )
    {
        double *p = &points[i*2];
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
