// C++ rewrite of https://github.com/tidwall/rtree.c

#pragma once

#include <cstdint>

#ifndef RTREE_ASSERT
#include <cassert>
#define RTREE_ASSERT assert
#endif

namespace fast_rtree
{
    typedef int32_t s32;

    static void* (*AllocFn)( size_t size );
    static void  (*FreeFn)( void* memory );

    enum NodeKind
    {
        Branch,
        Leaf,
    };

    // Returning false stops the iteration
    using Iterator = bool( const float min[], const float max[], const void* itemData, void* userdata );

    // TODO Find a good default for max/min items after some testing. Check benchmark results!
    // TODO Should we make this intrusive instead?
    template <typename ItemData, int Dims = 2, int MaxItems = 64, int MinItems = MaxItems / 10 + 1>
    struct RTree
    {
        static constexpr int MaxItemCount = MaxItems;
        static constexpr int MinItemCount = MinItems;
        static_assert( MinItems > 1 && MinItems <= MaxItems / 2, "Minimum items per node outside valid range" );

        struct Rect
        {
            float min[Dims];
            float max[Dims];
        };

        struct Node
        {
            Rect rects[MaxItems];
            union
            {
                Node* nodes[MaxItems];
                ItemData items[MaxItems];
            };
            s32 count;
            NodeKind kind;
        };


        Node* root;
        Rect rootRect;
        s32 height;
        s32 count;

        RTree()
            : root( nullptr )
            , rootRect{}
            , height( 0 )
            , count( 0 )
        {
            if( !AllocFn || !FreeFn )
            {
                AllocFn = malloc;
                FreeFn = free;
            }
        }

        // Returns false if failed (out of memory)
        bool Insert( ItemData const& item, float* min, float* max );
        void Search( const float min[], const float max[], Iterator* it, void* userdata ) const;

    private:
        // TODO Turn as many of these as possible into free funcs (will probably need to extract Node & Rect from the class)
        static Node* AllocNode( NodeKind kind );

        int FindBestNode( Node* node, Rect const& rect );
        bool SplitNodeByLargestAxisEdgeSnap( Rect const& rect, Node* node, Node** rightOut );
        bool SplitNode( Rect const& rect, Node* node, Node** rightOut );
        bool InsertNode( Node* node, Rect const& rect, ItemData const& item, bool* split );
        bool SearchNode( Node const* node, Rect const& rect, Iterator* it, void* userdata ) const;

        void MoveItem( Node *from, int index, Node *into );
        void SwapItem( Node* node, int i, int j );
        void NodeQSort( Node *node, int s, int e, int index );
        void SortNodeByAxis( Node* node, int axis, bool max );

        Rect ComputeRectFor( Node const* node );
        void ExpandRect( Rect *rect, Rect const& other );
        bool ContainsRect( Rect const& rect, Rect const& other );
        bool Intersects( Rect const& rect, Rect const& other ) const;
        float RectArea( Rect const& rect );
        float ExpandedRectArea( Rect const& rect, Rect const& other );
        int RectLargestAxis( Rect const& rect );

    };


    //// Implementation

#ifdef FAST_RTREE_IMPLEMENTATION

    // TODO Force inline?
    inline float Min( float x, float y )
    {
        return x < y ? x : y;
    }

    inline float Max( float x, float y )
    {
        return x > y ? x : y;
    }

#define TemplateDecl template <typename ItemData, int Dims, int MaxItems, int MinItems>
#define RTreeType RTree<ItemData, Dims, MaxItems, MinItems>

    template <typename ItemData, int Dims, int MaxItems, int MinItems>
    typename RTreeType::Node* RTreeType::AllocNode( NodeKind kind )
    {
        Node* node = (Node*)AllocFn( sizeof(Node) );
        if( node )
        {
            *node = {};
            node->kind = kind;
            return node;
        }
        return nullptr;
    }

    template <typename ItemData, int Dims, int MaxItems, int MinItems>
    typename RTreeType::Rect RTreeType::ComputeRectFor( Node const* node )
    {
        Rect rect = node->rects[0];
        int count = node->count;
        for( int i = 1; i < count; i++ )
            ExpandRect( &rect, node->rects[i] );

        return rect;
    }

    template <typename ItemData, int Dims, int MaxItems, int MinItems>
    void RTreeType::ExpandRect( Rect *rect, Rect const& other )
    {
        for( int i = 0; i < Dims; i++ )
        {
            rect->min[i] = Min( rect->min[i], other.min[i] );
            rect->max[i] = Max( rect->max[i], other.max[i] );
        }
    }

    template <typename ItemData, int Dims, int MaxItems, int MinItems>
    bool RTreeType::ContainsRect( Rect const& rect, Rect const& other )
    {
        int bits = 0;
        for( int i = 0; i < Dims; i++ )
        {
            bits |= other.min[i] < rect.min[i];
            bits |= other.max[i] > rect.max[i];
        }
        return bits == 0;
    }

    template <typename ItemData, int Dims, int MaxItems, int MinItems>
    float RTreeType::RectArea( Rect const& rect )
    {
        float result = 1.f;
        for( int i = 0; i < Dims; i++ )
            result *= (rect.max[i] - rect.min[i]);

        return result;
    }

    template <typename ItemData, int Dims, int MaxItems, int MinItems>
    float RTreeType::ExpandedRectArea( Rect const& rect, Rect const& other )
    {
        float result = 1.f;
        for( int i = 0; i < Dims; i++ )
            result *= (Max( rect.max[i], other.max[i] ) - Min( rect.min[i], other.min[i] ));

        return result;
    }

    template <typename ItemData, int Dims, int MaxItems, int MinItems>
    int RTreeType::RectLargestAxis( Rect const& rect )
    {
        int axis = 0;
        float maxLength = rect.max[0] - rect.min[0];
        for( int i = 1; i < Dims; i++ )
        {
            float length = rect.max[i] - rect.min[i];
            if( length > maxLength )
            {
                maxLength = length;
                axis = i;
            }
        }
        return axis;
    }


    template <typename ItemData, int Dims, int MaxItems, int MinItems>
    bool RTreeType::Insert( ItemData const& item, float* min, float* max )
    {
        Rect rect;
        memcpy( &rect.min[0], min, Dims * sizeof(float) );
        memcpy( &rect.max[0], max, Dims * sizeof(float) );

        while( true )
        {
            // If no root node yet, create one
            if( !root )
            {
                Node* newRoot = AllocNode( Leaf );
                if( !newRoot )
                    break;

                root = newRoot;
                rootRect = rect;
                height = 1;
            }

            bool split = false;
            if( !InsertNode( root, rect, item, &split ) )
                break;

            if( !split )
            {
                ExpandRect( &rootRect, rect );
                count++;
                return true;
            }

            Node* right;
            if( !SplitNode( rootRect, root, &right ) )
                break;

            Node* newRoot = AllocNode( Branch );
            if( !newRoot )
                break;

            newRoot->rects[0] = ComputeRectFor( root );
            newRoot->rects[1] = ComputeRectFor( right );
            newRoot->nodes[0] = root;
            newRoot->nodes[1] = right;
            newRoot->count = 2;

            root = newRoot;
            height++;
        }

        // OOM
        return false;
    }

    template <typename ItemData, int Dims, int MaxItems, int MinItems>
    int RTreeType::FindBestNode( Node* node, Rect const& rect )
    {
        int count = node->count;
        for( int i = 0; i < count; ++i )
        {
            if( ContainsRect( node->rects[i], rect ) )
                return i;
        }

        // Choose rect that will grow the least
        // TODO Test whether doing this in a single pass may be better overall?
        int bestIdx;
        float minExtraArea = FLT_MAX;
        for( int i = 0; i < count; ++i )
        {
            float area = RectArea( node->rects[i] );
            float newArea = ExpandedRectArea( node->rects[i], rect );
            float extra = newArea - area;
            if( extra < minExtraArea )
            {
                minExtraArea = extra;
                bestIdx = i;
            }
        }

        return bestIdx;
    }

    template <typename ItemData, int Dims, int MaxItems, int MinItems>
    void RTreeType::MoveItem( Node *from, int index, Node *into )
    {
        RTREE_ASSERT( into->count < MaxItems );

        into->rects[into->count] = from->rects[index];
        from->rects[index] = from->rects[from->count - 1];

        if( from->kind == Leaf )
        {
            into->items[into->count] = from->items[index];
            from->items[index] = from->items[from->count - 1];
        }
        else
        {
            into->nodes[into->count] = from->nodes[index];
            from->nodes[index] = from->nodes[from->count - 1];
        }

        from->count--;
        into->count++;
    }

    // swap two rectangles
    template <typename ItemData, int Dims, int MaxItems, int MinItems>
    void RTreeType::SwapItem( Node* node, int i, int j )
    {
        Rect tmp = node->rects[i];
        node->rects[i] = node->rects[j];
        node->rects[j] = tmp;

        if( node->kind == Leaf )
        {
            ItemData tmp = node->items[i];
            node->items[i] = node->items[j];
            node->items[j] = tmp;
        }
        else
        {
            Node* tmp = node->nodes[i];
            node->nodes[i] = node->nodes[j];
            node->nodes[j] = tmp;
        }
    }

    template <typename ItemData, int Dims, int MaxItems, int MinItems>
    void RTreeType::NodeQSort( Node *node, int s, int e, int index )
    { 
        int nrects = e - s;
        if( nrects < 2 )
            return;

        int left = 0;
        int right = nrects - 1;
        int pivot = nrects / 2;

        SwapItem( node, s + pivot, s + right );

        struct rect4 {
            float all[Dims * 2];
        };
        rect4* rects = (rect4*)&node->rects[s];
        for( int i = 0; i < nrects; i++ )
        {
            if( rects[right].all[index] < rects[i].all[index] )
            {
                SwapItem( node, s + i, s + left );
                left++;
            }
        }

        SwapItem( node, s + left, s + right );

        NodeQSort( node, s, s + left, index );
        NodeQSort( node, s + left + 1, e, index );
    }

    // Sort the node rectangles by the axis using quicksort. used during splits
    template <typename ItemData, int Dims, int MaxItems, int MinItems>
    void RTreeType::SortNodeByAxis( Node* node, int axis, bool max )
    {
        int byIndex = max ? Dims + axis : axis;
        NodeQSort( node, 0, node->count, byIndex );
    }

    template <typename ItemData, int Dims, int MaxItems, int MinItems>
    bool RTreeType::SplitNodeByLargestAxisEdgeSnap( Rect const& rect, Node* node, Node** rightOut )
    {
        int axis = RectLargestAxis( rect );

        Node* right = AllocNode( node->kind );
        if( !right )
            return false;

        // Check the remaining count every iteration
        for( int i = 0; i < node->count; ++i )
        {
            float min = node->rects[i].min[axis] - rect.min[axis];
            float max = rect.max[axis] - node->rects[i].max[axis];
            if( max < min )
            {
                MoveItem( node, i, right );
                i--;
            }
        }

        // Make sure that both left and right nodes have at least MinItems
        // TODO Test ways of avoiding these sorts
        if( node->count < MinItems )
        {
            // Reverse sort by min axis
            SortNodeByAxis( right, axis, false );
            do
            {
                MoveItem( right, right->count - 1, node );
            } while( node->count < MinItems );
        }
        else if( right->count < MinItems )
        {
            // Reverse sort by max axis
            SortNodeByAxis( node, axis, true );
            do
            {
                MoveItem( node, node->count - 1, right );
            } while( right->count < MinItems );
        }

        // TODO Wth does this do exactly?
        if( node->kind == Branch )
        {
            SortNodeByAxis( node, 0, false );
            SortNodeByAxis( right, 0, false );
        }

        *rightOut = right;
        return true;
    }

    template <typename ItemData, int Dims, int MaxItems, int MinItems>
    bool RTreeType::SplitNode( Rect const& rect, Node* node, Node** rightOut )
    {
        // TODO Test various strategies
        return SplitNodeByLargestAxisEdgeSnap( rect, node, rightOut );
    }

    // TODO Use the CopyOnWrite model of rtree.c as a means for supporting concurrent queries & modifications?
    template <typename ItemData, int Dims, int MaxItems, int MinItems>
    bool RTreeType::InsertNode( Node* node, Rect const& rect, ItemData const& item, bool* split )
    {
        if( node->kind == Leaf )
        {
            if( node->count == MaxItems )
            {
                *split = true;
                return true;
            }

            node->rects[ node->count ] = rect;
            node->items[ node->count ] = item;
            node->count++;
            *split = false;
            return true;
        }

        // Choose a subtree
        int n = FindBestNode( node, rect );
        if( !InsertNode( node->nodes[n], rect, item, split ) )
            return false;

        if( !*split )
        {
            ExpandRect( &node->rects[n], rect );
            return true;
        }

        // Split child node
        if( node->count == MaxItems )
        {
            *split = true;
            return true;
        }

        Node* right;
        if( !SplitNode( node->rects[n], node->nodes[n], &right ) )
            return false;

        node->rects[n] = ComputeRectFor( node->nodes[n] );
        node->rects[node->count] = ComputeRectFor( right );
        node->nodes[node->count] = right;
        node->count++;

        return InsertNode( node, rect, item, split );
    }


    //// Search

    TemplateDecl
    void RTreeType::Search( const float min[], const float max[], Iterator* it, void* userdata ) const
    {
        Rect rect;
        memcpy( &rect.min[0], min, sizeof(float) * Dims );
        memcpy( &rect.max[0], max, sizeof(float) * Dims );

        if( root )
            SearchNode( root, rect, it, userdata );
    }

    TemplateDecl
    bool RTreeType::SearchNode( Node const* node, Rect const& rect, Iterator* it, void* userdata ) const
    {
        if( node->kind == Leaf )
        {
            for( int i = 0; i < node->count; i++ )
            {
                if( Intersects( node->rects[i], rect ) )
                {
                    if( !it( node->rects[i].min, node->rects[i].max, &node->items[i], userdata ) )
                        return false;
                }
            }
            return true;
        }
        else
        {
            for( int i = 0; i < node->count; i++ )
            {
                if( Intersects( node->rects[i], rect ) )
                {
                    if( !SearchNode( node->nodes[i], rect, it, userdata ) )
                        return false;
                }
            }
            return true;
        }
    }

    TemplateDecl
    inline bool RTreeType::Intersects( Rect const& rect, Rect const& other ) const
    {
        // TODO SIMD
        // TODO Early out?
        int bits = 0;
        for( int i = 0; i < Dims; i++ )
        {
            bits |= other.min[i] > rect.max[i];
            bits |= other.max[i] < rect.min[i];
        }
        return bits == 0;
    }


#undef RTreeType

#endif // FAST_RTREE_IMPLEMENTATION

} // namespace fast_rtree

