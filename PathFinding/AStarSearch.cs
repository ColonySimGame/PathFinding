using System.Numerics;

namespace PathFindingGrid.PathFinding;

public class AStarSearch
{
    private readonly int _width;
    private readonly int _height;
    private readonly int _depth;
    private readonly Func<Point3D, bool> _isWalkable; // Function to check if a tile is walkable
    private readonly Dictionary<Point3D, List<Teleporter>> _teleporters;
    private readonly float _defaultMoveCost;
    private readonly float _diagonalMoveCost;
    private readonly bool _allowDiagonal;

    /// <summary>
    /// Initializes the A* search parameters.
    /// </summary>
    /// <param name="width">Map width (X dimension).</param>
    /// <param name="height">Map height (Y dimension).</param>
    /// <param name="depth">Map depth (Z dimension).</param>
    /// <param name="isWalkable">A function delegate that takes a Point3D and returns true if it's walkable, false otherwise.</param>
    /// <param name="teleporters">A dictionary mapping teleporter entry points to their exit points and traversal cost.</param>
    /// <param name="defaultMoveCost">Cost for orthogonal movement (up, down, left, right, forward, back).</param>
    /// <param name="diagonalMoveCost">Cost for diagonal movement on the same Z-level. Ignored if allowDiagonal is false.</param>
    /// <param name="allowDiagonal">Whether diagonal movement on the same Z-level is permitted.</param>
    public AStarSearch(
        int width,
        int height,
        int depth,
        Func<Point3D, bool> isWalkable,
        Dictionary<Point3D, List<Teleporter>>? teleporters = null,
        float defaultMoveCost = 1.0f,
        float diagonalMoveCost = 1.414f, // Approx sqrt(2)
        bool allowDiagonal = true)
    {
        _width = width;
        _height = height;
        _depth = depth;
        _isWalkable = isWalkable ?? throw new ArgumentNullException(nameof(isWalkable));
        _teleporters = teleporters ?? new Dictionary<Point3D, List<Teleporter>>();
        _defaultMoveCost = defaultMoveCost;
        _diagonalMoveCost = diagonalMoveCost;
        _allowDiagonal = allowDiagonal;
    }

    /// <summary>
    /// Finds the shortest path between start and goal points.
    /// </summary>
    /// <param name="startPoint">The starting position.</param>
    /// <param name="goalPoint">The target position.</param>
    /// <returns>A list of Point3D representing the path, or null if no path is found.</returns>
    public List<Point3D>? FindPath(Point3D startPoint, Point3D goalPoint)
    {
        // Basic validation
        if (!IsWithinBounds(startPoint) || !_isWalkable(startPoint) ||
            !IsWithinBounds(goalPoint) || !_isWalkable(goalPoint))
        {
            Console.WriteLine("Start or Goal point is invalid (out of bounds or unwalkable).");
            return null; // Invalid start or goal
        }

        if (startPoint == goalPoint)
        {
            return [startPoint];
        }

        // Use PriorityQueue for efficient retrieval of the node with the lowest FScore
        var openSet = new PriorityQueue<Node, float>();
        // Keep track of nodes already evaluated or in the open set to avoid duplicates/reprocessing
        var nodeLookup = new Dictionary<Point3D, Node>();
        // Store the GScore for quick lookup and update
        var gScores = new Dictionary<Point3D, float>();

        // Initialize starting node
        var startHScore = Heuristic(startPoint, goalPoint);
        var startNode = new Node(startPoint, 0, startHScore);
        openSet.Enqueue(startNode, startNode.FScore);
        nodeLookup[startPoint] = startNode;
        gScores[startPoint] = 0;


        while (openSet.Count > 0)
        {
            var currentNode = openSet.Dequeue();

            // Goal reached?
            if (currentNode.Position == goalPoint)
            {
                return ReconstructPath(currentNode);
            }

            // Explore neighbors
            foreach (var neighborInfo in GetNeighbors(currentNode.Position))
            {
                var neighborPosition = neighborInfo.Position;
                var movementCost = neighborInfo.Cost;

                // Calculate tentative GScore
                var tentativeGScore = currentNode.GScore + movementCost;

                // Check if this path to the neighbor is better than any previous one
                if (!(tentativeGScore < gScores.GetValueOrDefault(neighborPosition, float.MaxValue)))
                {
                    continue;
                }
                
                // This path is better. Update/add the neighbor node.
                var hScore = Heuristic(neighborPosition, goalPoint);
                if (nodeLookup.TryGetValue(neighborPosition, out var neighborNode))
                {
                    // Node exists, update it (PriorityQueue doesn't have a direct update,
                    // so we enqueue a new entry. The old one will eventually be dequeued
                    // and ignored because its GScore won't match the improved gScores entry).
                    // This is a common way to handle updates with basic PQs.
                    neighborNode.Parent = currentNode;
                    neighborNode.GScore = tentativeGScore;
                    neighborNode.HScore = hScore; // HScore might be constant, but recalculate for safety
                    openSet.Enqueue(neighborNode, neighborNode.FScore); // Enqueue with updated priority
                }
                else
                {
                    // New node encountered
                    neighborNode = new Node(neighborPosition, tentativeGScore, hScore, currentNode);
                    openSet.Enqueue(neighborNode, neighborNode.FScore);
                    nodeLookup.Add(neighborPosition, neighborNode);
                }
                // Update the known best GScore for this position
                gScores[neighborPosition] = tentativeGScore;
            }
        }

        Console.WriteLine("No path found.");
        return null; // No path found
    }

    // --- 6. Heuristic Function (3D Manhattan Distance) ---
    private float Heuristic(Point3D a, Point3D b)
    {
        // Manhattan distance is a common, admissible heuristic for grid maps
        // (it never overestimates the cost).
        return Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y) + Math.Abs(a.Z - b.Z);

        // Optional: Euclidean distance (might perform better if diagonal movement is cheap/frequent)
        // return Vector3.Distance(new Vector3(a.X, a.Y, a.Z), new Vector3(b.X, b.Y, b.Z));
        // Note: Euclidean distance is admissible but potentially less efficient computationally.
    }

    // --- Helper: Get Neighbors (including teleporters) ---
    private IEnumerable<(Point3D Position, float Cost)> GetNeighbors(Point3D current)
    {
        // 1. Standard Moves (Orthogonal and Diagonal on the same Z-level)
        for (var dx = -1; dx <= 1; dx++)
        {
            for (var dy = -1; dy <= 1; dy++)
            {
                // Skip the center point itself
                if (dx == 0 && dy == 0) continue;

                // Skip diagonal if not allowed
                if (!_allowDiagonal && Math.Abs(dx) + Math.Abs(dy) > 1) continue;

                var neighbor = new Point3D(current.X + dx, current.Y + dy, current.Z);

                // Check bounds and walkability
                if (IsWithinBounds(neighbor) && _isWalkable(neighbor))
                {
                    var cost = (Math.Abs(dx) + Math.Abs(dy) > 1) ? _diagonalMoveCost : _defaultMoveCost;
                    yield return (neighbor, cost);
                }
            }
        }


        // 2. Teleporter Moves
        if (_teleporters.TryGetValue(current, out List<Teleporter>? teleportInfo))
        {
            foreach (var teleporter in teleportInfo)
            {
                var exitPoint = teleporter.PointA == current ? teleporter.PointB : teleporter.PointA;
                var teleportCost = teleporter.Cost;

                // Ensure the exit point is valid before yielding
                if (IsWithinBounds(exitPoint) && _isWalkable(exitPoint))
                {
                    yield return (exitPoint, teleportCost);
                }
            }
        }
    }

    // --- Helper: Check Map Bounds ---
    private bool IsWithinBounds(Point3D p)
    {
        return p.X >= 0 && p.X < _width &&
               p.Y >= 0 && p.Y < _height &&
               p.Z >= 0 && p.Z < _depth;
    }

    // --- Helper: Reconstruct Path from Goal Node ---
    private List<Point3D> ReconstructPath(Node goalNode)
    {
        var path = new List<Point3D>();
        var current = goalNode;
        while (current != null)
        {
            path.Add(current.Position);
            current = current.Parent;
        }
        path.Reverse(); // Reverse to get path from start to goal
        return path;
    }
}