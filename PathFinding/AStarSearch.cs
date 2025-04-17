using System.Numerics;
using PathFindingGrid.Grid;

namespace PathFindingGrid.PathFinding;

public class AStarSearch
{
    private const float DefaultMoveCost = 1;
    private const float DiagonalMoveCost = 1.414f;
    
    private readonly int _width;
    private readonly int _height;
    private readonly int _depth;
    private readonly Map _map;
    private readonly bool _allowDiagonal;

    /// <summary>
    /// Initializes the A* search parameters.
    /// </summary>
    /// <param name="width">Map width (X dimension).</param>
    /// <param name="height">Map height (Y dimension).</param>
    /// <param name="depth">Map depth (Z dimension).</param>
    /// <param name="map"></param>
    /// <param name="allowDiagonal">Whether diagonal movement on the same Z-level is permitted.</param>
    public AStarSearch(
        int width,
        int height,
        int depth,
        Map map,
        bool allowDiagonal = true)
    {
        _width = width;
        _height = height;
        _depth = depth;
        _map = map;
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
        if (!IsWithinBounds(startPoint) || !IsWalkable(startPoint) || !IsWithinBounds(goalPoint) || !IsWalkable(goalPoint))
        {
            Console.WriteLine("Start or Goal point is invalid (out of bounds or unwalkable).");
            return null; // Invalid start or goal
        }

        if (startPoint == goalPoint)
        {
            return [startPoint];
        }

        // Use PriorityQueue for efficient retrieval of the node with the lowest FScore
        var openSet = new PriorityQueue<Node, double>();
        // Keep track of nodes already evaluated or in the open set to avoid duplicates/reprocessing
        var nodeLookup = new Dictionary<Point3D, Node>();
        // Store the GScore for quick lookup and update
        var gScores = new Dictionary<Point3D, double>();

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
    private IEnumerable<(Point3D Position, double Cost)> GetNeighbors(Point3D current)
    {
        // 1. Standard Moves (Orthogonal and Diagonal on the same Z-level)
        for (var dx = -1; dx <= 1; dx++)
        {
            for (var dy = -1; dy <= 1; dy++)
            {
                // Skip the center point itself
                if (dx == 0 && dy == 0)
                {
                    continue;
                }

                // Skip diagonal if not allowed
                if (!_allowDiagonal && Math.Abs(dx) + Math.Abs(dy) > 1)
                {
                    continue;
                }
                
                var neighbor = new Point3D(current.X + dx, current.Y + dy, current.Z);

                // Check bounds and walkability
                if (IsWithinBounds(neighbor) && IsWalkable(neighbor))
                {
                    // double cost = Math.Abs(dx) + Math.Abs(dy) > 1 ? _diagonalMoveCost : _defaultMoveCost;
                    // cost /= _map.Tiles[neighbor.X, neighbor.Y, neighbor.Z].SpeedModifier; // Adjust cost based on tile movement cost
                    yield return (neighbor, Cost(dx, dy, tileSpeedModifier: _map.Tiles[neighbor.X, neighbor.Y, neighbor.Z].SpeedModifier));
                }
            }
        }


        // 2. Teleporter Moves
        if (_map.TeleportersByPoint.TryGetValue(current, out List<Teleporter>? teleportInfo))
        {
            foreach (var teleporter in teleportInfo)
            {
                var exitPoint = teleporter.PointA == current ? teleporter.PointB : teleporter.PointA;
                var teleportCost = teleporter.Cost;

                // Ensure the exit point is valid before yielding
                if (IsWithinBounds(exitPoint) && IsWalkable(exitPoint))
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
    
    private bool IsWalkable(Point3D p)
    {
        if (p.X < 0 || p.X >= _width || p.Y < 0 || p.Y >= _height || p.Z < 0 || p.Z >= _depth)
        {
            return false;
        }
        
        return _map.Tiles[p.X, p.Y, p.Z].Walkable;
    }

    public static double CalculateMoveCost(Point3D from, Point3D to, Dictionary<Point3D, List<Teleporter>> teleporters, Tile tile)
    {
        // Check teleporter first
        if (teleporters.TryGetValue(from, out List<Teleporter>? teleportInfo))
        {
            foreach (var teleporter in from teleporter in teleportInfo let isTeleporter = teleporter.PointA == to || teleporter.PointB == to where isTeleporter select teleporter)
            {
                return teleporter.Cost;
            }
        }

        // Assume standard movement cost otherwise (this is simplified)
        var dx = Math.Abs(from.X - to.X);
        var dy = Math.Abs(from.Y - to.Y);
        var dz = Math.Abs(from.Z - to.Z);

        if (dz > 0) {
            // Need logic for Z-movement cost if implemented (e.g., stairs cost)
            return float.PositiveInfinity; // Placeholder for undefined Z-move cost
        }

        if (dx <= 1 && dy <= 1) // Adjacent or diagonal on same plane
        {
            return Cost(dx, dy, tile.SpeedModifier, 1);
        }

        // This indicates a jump or non-standard move not covered by teleporter/adjacent
        return float.MaxValue; // Or handle appropriately
    }
    
    private static double Cost(int dx, int dy, double tileSpeedModifier = 1, double characterModifier = 1)
    {
        double cost = dx + dy > 1 ? DiagonalMoveCost : DefaultMoveCost; // Diagonal vs Orthogonal
        // Adjust cost based on tile movement cost (if applicable)
        cost /= tileSpeedModifier;
        // cost *= 60;
        cost /= characterModifier;
        return cost;
    }
}