using PathFindingGrid.Grid;
using PathFindingGrid.PathFinding;

namespace PathFindingGrid;

public abstract class Program
{
    public static void Main(string[] args)
    {
        const int width = 250;
        const int height = 250;
        const int depth = 10; // 3 Z-Levels (0, 1, 2)
        
        var map = new Map(new Tile[width, height, depth]);
        
        // Initialize map (example: all walkable)
        for (var z = 0; z < depth; z++)
            for (var y = 0; y < height; y++)
                for (var x = 0; x < width; x++)
                    map.Tiles[x, y, z] = new Tile(new Point3D(x, y, z));

        // Add some obstacles
        map.Tiles[5, 5, 0].Walkable = false;
        map.Tiles[5, 6, 0].Walkable = false;
        map.Tiles[5, 7, 0].Walkable = false;
        map.Tiles[4, 6, 1].Walkable = false;
        map.Tiles[5, 6, 1].Walkable = false;
        map.Tiles[6, 6, 1].Walkable = false;
        
        // A wall from the top to the bottom with a hole at the top on z-level 1 on x = 5
        for (var y = 0; y < height; y++)
        {
            if (y != 0)
            {
                map.Tiles[10, y, 1].Walkable = false;
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
        foreach (var rawTeleporter in rawTeleporters)
        {
            map.AddTeleporter(rawTeleporter);
        }

        // --- Initialize A* ---
        var astar = new AStarSearch(width, height, depth, map, allowDiagonal: true);

        // --- Define Start and Goal ---
        var start = new Point3D(0, 0, 0);
        var goal = new Point3D(9, 9, 4);

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
            double totalCost = 0;
            startTime = DateTime.Now.Ticks;
            for(var i = 0; i < path.Count - 1; i++)
            {
                var origin = path[i];
                var target = path[i+1];
                var tile = map.GetTile(target);
                totalCost += AStarSearch.CalculateMoveCost(origin, target, map.TeleportersByPoint, tile);
            }
            endTime = DateTime.Now.Ticks;
            Console.WriteLine($"Time taken: {(endTime - startTime) / 10000} ms");
            Console.WriteLine($"Approximate Path Cost: {totalCost}"); // Note: This simple calc might slightly differ from A*'s internal G-score due to heuristic influence/tie-breaking.
        }
        else
        {
            Console.WriteLine("Path could not be found.");
        }
    }
    
}