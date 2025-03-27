using PathFindingGrid.Grid;

namespace PathFindingGrid.PathFinding;

public class Node : IComparable<Node>
{
    public Point3D Position { get; }
    public float GScore { get; set; } // Cost from start to this node
    public float HScore { get; set; } // Heuristic cost from this node to goal
    public float FScore => GScore + HScore; // Total estimated cost
    public Node? Parent { get; set; }

    public Node(Point3D position, float gScore = float.MaxValue, float hScore = float.MaxValue, Node? parent = null)
    {
        Position = position;
        GScore = gScore;
        HScore = hScore;
        Parent = parent;
    }

    // Compare nodes based on FScore for Priority Queue
    public int CompareTo(Node? other)
    {
        if (other is null) return 1;
        var compare = FScore.CompareTo(other.FScore);
        if (compare == 0) // Tie-breaker using HScore (optional, can help explore promising paths first)
        {
            compare = HScore.CompareTo(other.HScore);
        }
        return compare;
        // Note: For PriorityQueue<TElement, TPriority>, we just need the priority (FScore)
        // when enqueueing. This CompareTo is more for SortedSet or manual sorting.
    }
}