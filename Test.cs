using PathFindingGrid.PathFinding;

namespace PathFindingGrid;

using System;
using System.Collections.Generic;
using System.Linq;

// --- Example Usage ---
public class Example
{
    public static void Main(string[] args)
    {
        // --- 3. Map Representation ---
        const int width = 250;
        const int height = 250;
        const int depth = 10; // 3 Z-Levels (0, 1, 2)
        var map = new bool[width, height, depth];

        // Initialize map (example: all walkable)
        for (var z = 0; z < depth; z++)
            for (var y = 0; y < height; y++)
                for (var x = 0; x < width; x++)
                    map[x, y, z] = true;

        // Add some obstacles
        map[5, 5, 0] = false;
        map[5, 6, 0] = false;
        map[5, 7, 0] = false;
        map[4, 6, 1] = false;
        map[5, 6, 1] = false;
        map[6, 6, 1] = false;
        
        // A wall from the top to the bottom with a hole at the top on z-level 1 on x = 5
        for (var y = 0; y < height; y++)
        {
            if (y != 0)
            {
                map[10, y, 1] = false;
            }
        }

        // --- 4. Define Teleporters ---
        var rawTeleporters = new List<Teleporter>()
        {
            new(new Point3D(1, 1, 0), new Point3D(8, 8, 1), 2.0f),
            new(new Point3D(30, 8, 2), new Point3D(9, 10, 2), 0.5f),
            new(new Point3D(0, 0, 0), new Point3D(0, 0, 1), 1.0f),
            new(new Point3D(36, 13, 1), new Point3D(36, 13, 2), 1.0f),
            new(new Point3D(0, 15, 2), new Point3D(0, 15, 3), 1.0f),
            new(new Point3D(5, 8, 4), new Point3D(5, 8, 3), 1.0f),
        };
        var teleporters = new Dictionary<Point3D, List<Teleporter>>();
        foreach (var teleporter in rawTeleporters)
        {
            teleporters.TryAdd(teleporter.PointA, []);
            teleporters[teleporter.PointA].Add(teleporter);
            
            teleporters.TryAdd(teleporter.PointB, []);
            teleporters[teleporter.PointB].Add(teleporter);
        }

        // --- Initialize A* ---
        var astar = new AStarSearch(width, height, depth, IsWalkableFunc, teleporters, allowDiagonal: true);

        // --- Define Start and Goal ---
        var start = new Point3D(0, 0, 0);
        // Point3D goal = new Point3D(9, 9, 0); // Goal on same level
         // Point3D goal = new Point3D(9, 9, 1); // Goal on different level, reachable via teleporter
        var goal = new Point3D(9, 9, 4); // Goal potentially reachable via Z-level changes (if implemented) or other teleporters


        Console.WriteLine($"Finding path from {start} to {goal}");

        // --- Find Path ---
        var startTime = DateTime.Now.Ticks;
        List<Point3D>? path = astar.FindPath(start, goal);
        var endTime = DateTime.Now.Ticks;
        Console.WriteLine($"Time taken: {(endTime - startTime) / 10000} ms");

        // --- Output Path ---
        if (path is not null)
        {
            Console.WriteLine("Path found:");
            Console.WriteLine(string.Join(" -> ", path));
            // Calculate path cost (optional)
            float totalCost = 0;
            for(var i = 0; i < path.Count - 1; i++) {
                totalCost += CalculateMoveCost(path[i], path[i+1], teleporters);
            }
            Console.WriteLine($"Approximate Path Cost: {totalCost}"); // Note: This simple calc might slightly differ from A*'s internal G-score due to heuristic influence/tie-breaking.
        }
        else
        {
            Console.WriteLine("Path could not be found.");
        }

        return;

        // --- Create IsWalkable Function ---
        // This lambda captures the 'map' variable
        bool IsWalkableFunc(Point3D p)
        {
            // Check bounds implicitly via AStarSearch constructor, but double-checking is safe
            if (p.X < 0 || p.X >= width || p.Y < 0 || p.Y >= height || p.Z < 0 || p.Z >= depth)
            {
                return false;
            }

            return map[p.X, p.Y, p.Z];
        }
    }

     // Helper function for example cost calculation (mirrors neighbor logic)
    private static float CalculateMoveCost(Point3D from, Point3D to, Dictionary<Point3D, List<Teleporter>> teleporters)
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
            return (dx + dy > 1) ? 1.414f : 1.0f; // Diagonal vs Orthogonal
        }

        // This indicates a jump or non-standard move not covered by teleporter/adjacent
        return float.MaxValue; // Or handle appropriately
    }
}